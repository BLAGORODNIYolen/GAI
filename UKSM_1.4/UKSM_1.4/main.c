// ==================== main.c ====================
// Полная интегрированная программа для ATmega2560
// Модули: UART, PWM (с ПИ-регулятором), INPUT (кнопки, АЦП), FLASH (EEPROM)

#include "config.h"
#include "uart.h"
#include "pwm.h"
#include "input.h"
#include "flash.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>


// ==================== НАСТРОЙКИ ПРОЕКТА ====================
// Все настройки можно менять здесь, не затрагивая модули

// -------------------- Режимы работы --------------------



static rx_packet_t rx_packet;

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ MAIN ====================
// Состояние системы
static uint8_t system_mode;      // Текущий режим (TEST/WORK)
static uint16_t current_voltage_setpoint;  // Уставка напряжения (В)
static float current_voltage = 0.0f;            // Измеренное напряжение (В)
static float current_cal_adc_voltage_value = 0.0f; //текущий калибровочный коэффициент АЦП (В/единица)
static float current_cal_out_voltage_value = 0.0f; //текущий калибровочный коэффициент VOUT (В/единица)
static float current_cal_out_voltage_offset_value = 0.0f; //текущий второй калибровочный коэффициент VOUT (В/единица)
static uint16_t current_duty = 0;               // Текущая скважность (0-1000)
static uint8_t status = 0; //управляющее слово от мастера
static uint8_t error_flags = 0; 
// Биты ошибок: 0 - EEPROM,
// 1 - Overvoltage,
// 2 - Undervoltage, 
// 3- Setpoint error, 


static uint16_t rx_packet_counter = 0;
static uint16_t tx_packet_counter = 0;

static tx_packet_t ack_packet;
// Биты ошибок: 0 - EEPROM,
// 1 - Overvoltage,
// 2 - Undervoltage, 
// 3- Setpoint error, 
// 6 = 0 - ACK, 1 - NACK
// 7 - mode (0 - test, 1 - work)
static tx_packet_t nack_packet;
// Биты ошибок: 0 - UNKNOWN_COMMAND,
// 1 - UNKNOWN PARAMS,
// 2 - CHECKSUM ERROR, 
// 6 = 0 - ACK, 1 - NACK
// 7 - mode (0 - test, 1 - work)

// ==================== ПРОТОТИПЫ ФУНКЦИЙ MAIN ====================
static void init_system(void);
static void process_buttons(void);
static void process_uart(void);
static void update_measurements(void);
static void update_control(void);
static void send_periodic_data(void);
static void print_startup_info(void);

static void start_adc_calibration();
static void start_vout_calibration();

static void update_flash(void);
static void timer2_init(void);
uint16_t get_time_ms(void);
static void toggle_led(void);

static float fabs(float x) {
    return (x < 0) ? -x : x;
}
// ==================== ТАЙМЕР 2 ДЛЯ ПЕРИОДИЧЕСКИХ ЗАДАЧ ====================
// Используем Timer2 для создания базового такта 1 мс
// Это позволяет точно выдерживать интервалы 20 мс (ПИ) и 500 мс (отправка)

static volatile uint16_t system_tick_ms = 0;

static void timer2_init(void) {
    // Настройка Timer2 на прерывание каждую 1 мс
    // F_CPU = 16 MHz, предделитель 64 -> 16e6/64 = 250000 Гц
    // OCR2 = 249 -> 250000 / 250 = 1000 Гц (1 мс)
    TCCR2A = (1 << WGM21);  // Режим CTC
    TCCR2B = (1 << CS22);    // Предделитель 64
    OCR2A = 249;              // 1 мс
    TIMSK2 = (1 << OCIE2A);  // Включаем прерывание
    sei();                    // Глобально разрешаем прерывания
}

ISR(TIMER2_COMPA_vect) {
    system_tick_ms++;
}

uint16_t get_time_ms(void) {
    uint16_t tick;
    cli();
    tick = system_tick_ms;
    sei();
    return tick;
}


// ==================== ИНИЦИАЛИЗАЦИЯ ВСЕХ МОДУЛЕЙ ====================

static void init_system(void) {
    timer2_init(); // Запускаем таймер для периодических задач
    //вызов флеш- выгрузка адресов слейв, мастера, режима,версии,статуса,калибровочных коэффициентов
    uint8_t flash_init_result = flash_init();
    uart_init(flash_get_slave_addr(), flash_get_master_addr());

    if (flash_init_result) {
        error_flags &= ~(1 << EEPROM_ERROR); // Сброс флага ошибки EEPROM
        #ifdef DEBUG_ENABLE
        uart_send_debug("FLASH: Settings loaded from EEPROM");
        #endif
    } else {
        error_flags |= (1 << EEPROM_ERROR); // Установка флага ошибки EEPROM
        #ifdef DEBUG_ENABLE
        uart_send_debug("FLASH: No valid settings in EEPROM, defaults used");
        #endif
    }
    //Создаем копии переменных из flash необходимых для работы main.c
    // для того чтобы регулярно не перезаписывать память
    //необходимы current setpoint, system_mode, status(как байт ошибок)
    current_voltage_setpoint = flash_get_setpoint();
    system_mode = flash_get_mode();
    status = flash_get_status();
	current_cal_out_voltage_value = flash_get_cal_out_voltage_value();
	current_cal_out_voltage_offset_value = flash_get_cal_out_voltage_offset_value();
	current_cal_adc_voltage_value = flash_get_cal_adc_voltage_value();
	current_duty = 5;

    buttons_init();
    #ifdef DEBUG_ENABLE
    uart_send_debug("INPUT: Buttons initialized");
    #endif

    adc_init();
    #ifdef DEBUG_ENABLE
    uart_send_debug("INPUT: ADC initialized");
    #endif

    pwm_init();
    #ifdef DEBUG_ENABLE
    uart_send_debug("PWM: Initialized, output disabled");
    #endif

    #ifdef DEBUG_ENABLE
    print_startup_info();
    #endif
    pi_init();

    if (system_mode == MODE_WORK) {
        #ifdef DEBUG_ENABLE
        uart_send_debug("PI Regulator: Initialized");
        #endif
    } else {
        #ifdef DEBUG_ENABLE
        uart_send_debug("PI Regulator: Not used (Test mode)");
        #endif
        pwm_set_duty(current_duty);
        #ifdef DEBUG_ENABLE
        uart_send_debug("Default Duty(5) is set by buttons");
        #endif
    }


    uart_async_enable();
    #ifdef DEBUG_ENABLE
    uart_send_debug("UART: Async receive enabled");
    uart_send_debug("\n\r========================================\r\n\r\n");
    #endif

    
    //запустить ШИМ с соответствующей уставкой

}

static void print_startup_info(void) {
    uart_send_debug("\r\n--- System Configuration ---\r\n");
    uart_send_string("Version: ");
    uart_send_debug_num(DEC, (uint16_t) flash_get_version());
    uart_send_string("\nSlave Addr: ");
    uart_send_debug_num(HEX, (uint16_t) flash_get_slave_addr());
    uart_send_string(", Master Addr: ");
    uart_send_debug_num(HEX, (uint16_t) flash_get_master_addr());
    uart_send_string("Mode: ");
    if (flash_get_mode() == MODE_TEST) {
        uart_send_string("TEST\r\n");
    } else {
        uart_send_string("WORK\r\n");
    }
    uart_send_string("Current Status: ");
    uart_send_debug_num(BIN, (uint16_t) flash_get_status());
    uart_send_string("\nCalibration ADC Voltage: ");
    uart_send_debug_float(flash_get_cal_adc_voltage_value());
    uart_send_string(" V/count");
    uart_send_string("\nCalibration OUT Voltage: ");
    uart_send_debug_float(flash_get_cal_out_voltage_value());
    uart_send_string(" V/count");
    uart_send_string("\nCalibration OUT Voltage Offset: ");
    uart_send_debug_float(flash_get_cal_out_voltage_offset_value());
    uart_send_string(" V");
    uart_send_string("\nInitial Voltage Setpoint: ");
    uart_send_debug_num(DEC, (uint16_t) flash_get_setpoint());
    uart_send_debug("\r\n----------------------------\r\n");
}

// ==================== ОБРАБОТКА КНОПОК ====================

static void process_buttons(void) {
    uint8_t up = if_button_up_was_pressed();
    uint8_t down = if_button_down_was_pressed();
    
    if (!up && !down) return;
    
    if (system_mode == MODE_WORK) {
        // Рабочий режим: кнопки меняют уставку напряжения
        if (up) {
            if (current_voltage_setpoint < VOLTAGE_MAX - VOLTAGE_STEP_BUTTON) {
                current_voltage_setpoint += VOLTAGE_STEP_BUTTON;
            } else {
                current_voltage_setpoint = (uint16_t)VOLTAGE_MAX;
            }
            #ifdef DEBUG_ENABLE
            uart_send_string("Button: Setpoint increased to ");
            uart_send_debug_num(DEC, current_voltage_setpoint);
			uart_send_debug(";");
            #endif

        }
        
        if (down) {
            if (current_voltage_setpoint > VOLTAGE_MIN + VOLTAGE_STEP_BUTTON) {
                current_voltage_setpoint -= VOLTAGE_STEP_BUTTON;
            } else {
                current_voltage_setpoint = (uint16_t)VOLTAGE_MIN;
            }
            #ifdef DEBUG_ENABLE
            uart_send_string("Button: Setpoint decreased to ");
            uart_send_debug_num(DEC, current_voltage_setpoint);
			uart_send_debug(";");
            #endif
        }
    } else {
		// Тестовый режим: кнопки меняют скважность напрямую
     
        if (up) {
            if (current_duty <= PWM_MAX_AVAILABLE_DUTY - DUTY_STEP_BUTTON) {
                current_duty += DUTY_STEP_BUTTON;
            } else {
                current_duty = PWM_MAX_DUTY;
            }
            #ifdef DEBUG_ENABLE
            uart_send_string("Button: Duty increased to ");
            uart_send_debug_num(DEC, current_duty);
            #endif
        }
        
        if (down) {
            if (current_duty >= DUTY_STEP_BUTTON) {
                current_duty -= DUTY_STEP_BUTTON;
            } else {
                current_duty = 0;
            }
            #ifdef DEBUG_ENABLE
            uart_send_string("Button: Duty decreased to ");
            uart_send_debug_num(DEC, current_duty);
            #endif
        }
        pwm_set_duty(current_duty);
    }
}

// ==================== ОБРАБОТКА UART ====================

/*
	команды управления от мастера
	определяется по значению status
	status:
	0x10 - изменение калибровочного коэффициента АЦП +
	0x20 - изменение калибровочного коэффициента VOUT +
	0x30 - запрос данных +
	0x40 - изменение режима работы +
	0x50 - старт калибровки АЦП +
	0x60 - старт калибровки VOUT +
	0x70 - изменение уставки напряжения +
    0xA0 - сброс настроек к заводским +	
*/
static void process_uart(void) {
    if (uart_async_was_packet_lost()) {
        error_flags |= (1 << UART_ERROR);
        #ifdef DEBUG_ENABLE
        uart_send_debug("UART: Packet lost in async buffer!");
        #endif
        nack_packet.start = 0xAA;
        nack_packet.master_addr = flash_get_master_addr();
        nack_packet.slave_addr = flash_get_slave_addr();
        nack_packet.packet_num = tx_packet_counter++;
        nack_packet.voltage = current_voltage; // текущее измерение
        nack_packet.setpoint = current_voltage_setpoint; // подтверждаем полученную уставку
        nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_PARITY);
        uart_send_packet(&nack_packet);
    }
    if (uart_async_has_packet()) {
        if (uart_async_get_packet(&rx_packet)) {
                #ifdef DEBUG_ENABLE
                uart_send_string("UART: Received packet # ");
                uart_send_debug_num(DEC, rx_packet.packet_num);
                uart_send_debug(";");
                #endif
			rx_packet_counter++;
			if (rx_packet.status == START_ADC_CALIBRATION_COMMAND) { //калибровка АЦП
				start_adc_calibration();
			}
			else if (rx_packet.status == START_VOUT_CALIBRATION_COMMAND) { //калибровка VOUT
				start_vout_calibration();
			}
            else if (rx_packet.status == EDIT_SETPOINT_COMMAND) { // обработка уставки
                if (rx_packet.setpoint >= (uint16_t)VOLTAGE_MIN &&
				   rx_packet.setpoint <= (uint16_t)VOLTAGE_MAX) {
					current_voltage_setpoint = rx_packet.setpoint;
					   flash_set_setpoint(current_voltage_setpoint);
					   #ifdef DEBUG_ENABLE
					   uart_send_string("UART: Setpoint updated to ");
					   uart_send_debug_num(DEC, current_voltage_setpoint);
					   #endif
                    ack_packet.start = 0xAA;
                    ack_packet.master_addr = rx_packet.master_addr;
                    ack_packet.slave_addr = flash_get_slave_addr();
                    ack_packet.packet_num = tx_packet_counter++;
                    ack_packet.voltage = current_voltage; // текущее измерение
                    ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                    ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
                    uart_send_packet(&ack_packet);
				} else {
					   #ifdef DEBUG_ENABLE
					   uart_send_string("UART: Invalid setpoint received: ");
					   uart_send_debug_num(DEC, rx_packet.setpoint);
					   #endif
                    nack_packet.start = 0xAA;
                    nack_packet.master_addr = rx_packet.master_addr;
                    nack_packet.slave_addr = flash_get_slave_addr();
                    nack_packet.packet_num = tx_packet_counter++;
                    nack_packet.voltage = current_voltage; // текущее измерение
                    nack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                    nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_PARAM);
                    uart_send_packet(&nack_packet);
				}
            }
            else if (rx_packet.status == MODE_TOGGLE_COMMAND) { // переключение режима
                if (rx_packet.setpoint == 1){//включить режим MODE_WORK
                    system_mode = MODE_WORK;
                    update_flash();
                    #ifdef DEBUG_ENABLE
                    uart_send_debug("UART: Mode toggled to WORK");
                    #endif
                } else if (rx_packet.setpoint == 0){//включить режим MODE_TEST
                    system_mode = MODE_TEST;
                    update_flash();
                    #ifdef DEBUG_ENABLE
                    uart_send_debug("UART: Mode toggled to TEST");
                    #endif
                }
                else{
                    #ifdef DEBUG_ENABLE
                    uart_send_string("UART: Invalid mode toggle value received: ");
                    uart_send_debug_num(DEC, rx_packet.setpoint);
                    #endif
                    nack_packet.start = 0xAA;
                    nack_packet.master_addr = rx_packet.master_addr;
                    nack_packet.slave_addr = flash_get_slave_addr();
                    nack_packet.packet_num = tx_packet_counter++;
                    nack_packet.voltage = current_voltage; // текущее измерение
                    nack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                    nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_PARAM);
                    uart_send_packet(&nack_packet);
                    return;
                }
                ack_packet.start = 0xAA;
                ack_packet.master_addr = rx_packet.master_addr;
                ack_packet.slave_addr = flash_get_slave_addr();
                ack_packet.packet_num = tx_packet_counter++;
                ack_packet.voltage = current_voltage; // текущее измерение
                ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;// бит 6 = ACK
                uart_send_packet(&ack_packet);
            }
            else if (rx_packet.status == RESET_COMMAND) { // сброс настроек к заводским
                flash_reset_to_defaults();
                current_voltage_setpoint = flash_get_setpoint();
                system_mode = flash_get_mode();
                status = flash_get_status();
                current_cal_adc_voltage_value = flash_get_cal_adc_voltage_value();
                current_cal_out_voltage_value = flash_get_cal_out_voltage_value();
                current_cal_out_voltage_offset_value = flash_get_cal_out_voltage_offset_value();
                update_flash(); // чтобы сбросить флаг ошибки EEPROM, если он был
                #ifdef DEBUG_ENABLE
                uart_send_debug("UART: Settings reset to defaults");
                #endif
                ack_packet.start = 0xAA;
                ack_packet.master_addr = rx_packet.master_addr;
                ack_packet.slave_addr = flash_get_slave_addr();
                ack_packet.packet_num = tx_packet_counter++;
                ack_packet.voltage = current_voltage; // текущее измерение
                ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;// бит 6 = ACK
                uart_send_packet(&ack_packet);
            }
            else if (rx_packet.status == GET_DATA_COMMAND) { // запрос данных
                ack_packet.start = 0xAA;
                ack_packet.master_addr = rx_packet.master_addr;
                ack_packet.slave_addr = flash_get_slave_addr();
                ack_packet.packet_num = tx_packet_counter++;
                ack_packet.voltage = current_voltage; // текущее измерение
                ack_packet.setpoint = current_voltage_setpoint; // подтверждаем текущую уставку
                ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;// бит 6 = ACK
                uart_send_packet(&ack_packet);
            }
            // если команда гавно
           else{ 
                error_flags |= (1 << UART_ERROR); // Установка флага ошибки UART
                    #ifdef DEBUG_ENABLE
                    uart_send_debug("UART: Invalid packet received");
                    #endif
                nack_packet.start = 0xAA;
                nack_packet.master_addr = rx_packet.master_addr;
                nack_packet.slave_addr = flash_get_slave_addr();
                nack_packet.packet_num = tx_packet_counter++;
                nack_packet.voltage = current_voltage; // текущее измерение
                nack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
                nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
                uart_send_packet(&nack_packet);
                return;
			}
            error_flags &= ~(1 << UART_ERROR); // Сброс флага ошибки UART, если команда была обработана успешно
        }
        else {            // если пакет гавно
            error_flags |= (1 << UART_ERROR); // Установка флага ошибки UART
            #ifdef DEBUG_ENABLE
            uart_send_debug("UART: Error receiving packet");
            #endif
            nack_packet.start = 0xAA;
            nack_packet.master_addr = rx_packet.master_addr;
            nack_packet.slave_addr = flash_get_slave_addr();
            nack_packet.packet_num = tx_packet_counter++;
            nack_packet.voltage = current_voltage; 
            nack_packet.setpoint = rx_packet.setpoint; 
            nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
            uart_send_packet(&nack_packet);
        }
    }
}
static void update_control(void) {
	if (system_mode == MODE_WORK) {
		// ПИ-регулятор по напряжению
		current_duty = pi_compute(current_voltage_setpoint, current_voltage);
		pwm_set_duty(current_duty);
	}
}



static void start_adc_calibration(void){
    ack_packet.start = 0xAA;
    ack_packet.master_addr = rx_packet.master_addr;
    ack_packet.slave_addr = flash_get_slave_addr();
    ack_packet.packet_num = tx_packet_counter++;
    ack_packet.voltage = current_voltage; // текущее измерение
    ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
    ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
    uart_send_packet(&ack_packet);
	current_duty = 50;
	pwm_set_duty(current_duty);
	#ifdef DEBUG_ENABLE
	uart_send_debug("waiting for cal_adc_voltage...");
	#endif
	while(!uart_async_has_packet()){}
	if (uart_async_get_packet(&rx_packet)){
		if (rx_packet.status & EDIT_ADC_CALIBRATION_COMMAND) {//изменение калибровочного коэффицента АЦП
			float val = (float)rx_packet.setpoint / 1000.0f;
			current_cal_adc_voltage_value = val/(float)get_output_adc_channel();
			flash_set_cal_adc_voltage_value(current_cal_adc_voltage_value);
			update_calibration_values();
			#ifdef DEBUG_ENABLE
			uart_send_debug("ADC calibration saved");
			uart_send_string("Current adc coef: ");
			uart_send_debug_float(flash_get_cal_adc_voltage_value());
			uart_send_debug(" V/count;");
			#endif
            ack_packet.start = 0xAA;
            ack_packet.master_addr = rx_packet.master_addr;
            ack_packet.slave_addr = flash_get_slave_addr();
            ack_packet.packet_num = tx_packet_counter++;
            ack_packet.voltage = current_voltage; // текущее измерение
            ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
            ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
            uart_send_packet(&ack_packet);

		}else{
            nack_packet.start = 0xAA;
            nack_packet.master_addr = rx_packet.master_addr;
            nack_packet.slave_addr = flash_get_slave_addr();
            nack_packet.packet_num = tx_packet_counter++;
            nack_packet.voltage = current_voltage; 
            nack_packet.setpoint = rx_packet.setpoint; 
            nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
			error_flags |= UART_ERROR;
			#ifdef DEBUG_ENABLE
			uart_send_debug("error: cal_packet's been lost");
			#endif
            uart_send_packet(&nack_packet);
		}
	}else{
		error_flags |= UART_ERROR;
		#ifdef DEBUG_ENABLE
		uart_send_debug("error: cal_packet's been lost");
		#endif
        nack_packet.start = 0xAA;
        nack_packet.master_addr = rx_packet.master_addr;
        nack_packet.slave_addr = flash_get_slave_addr();
        nack_packet.packet_num = tx_packet_counter++;
        nack_packet.voltage = current_voltage; 
        nack_packet.setpoint = rx_packet.setpoint; 
        nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
        uart_send_packet(&nack_packet);
	}

}

static void start_vout_calibration(void){
    static float y1, x1, y2,x2;
    ack_packet.start = 0xAA;
    ack_packet.master_addr = rx_packet.master_addr;
    ack_packet.slave_addr = flash_get_slave_addr();
    ack_packet.packet_num = tx_packet_counter++;
    ack_packet.voltage = current_voltage; // текущее измерение
    ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
    ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
    uart_send_packet(&ack_packet);
    //первая точка для калибровки
	current_duty = 15;
	pwm_set_duty(current_duty);
	#ifdef DEBUG_ENABLE
	uart_send_debug("waiting for cal_vout_voltage...");
	#endif
	while(!uart_async_has_packet()){}
	if (uart_async_get_packet(&rx_packet)){
		if (rx_packet.status & EDIT_VOUT_CALIBRATION_COMMAND) {//изменение калибровочного коэффицента VOUT
			y1 = (float)rx_packet.setpoint / 100.0f;
            for (uint8_t i = 0; i < 50; i++){
                x1 = get_output_adc_channel();
            }
		}else{
            error_flags |= UART_ERROR;
            #ifdef DEBUG_ENABLE
            uart_send_debug("error: cal_packet's been lost");
            #endif
            nack_packet.start = 0xAA;
            nack_packet.master_addr = rx_packet.master_addr;
            nack_packet.slave_addr = flash_get_slave_addr();
            nack_packet.packet_num = tx_packet_counter++;
            nack_packet.voltage = current_voltage; 
            nack_packet.setpoint = rx_packet.setpoint; 
            nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
            uart_send_packet(&nack_packet);
            return;
		}
	}else{
		error_flags |= UART_ERROR;
		#ifdef DEBUG_ENABLE
		uart_send_debug("error: cal_packet's been lost");
		#endif
        nack_packet.start = 0xAA;
        nack_packet.master_addr = rx_packet.master_addr;
        nack_packet.slave_addr = flash_get_slave_addr();
        nack_packet.packet_num = tx_packet_counter++;
        nack_packet.voltage = current_voltage; 
        nack_packet.setpoint = rx_packet.setpoint; 
        nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
        uart_send_packet(&nack_packet);
        return;
	}
    //получили первую точку
    ack_packet.start = 0xAA;
    ack_packet.master_addr = rx_packet.master_addr;
    ack_packet.slave_addr = flash_get_slave_addr();
    ack_packet.packet_num = tx_packet_counter++;
    ack_packet.voltage = current_voltage; // текущее измерение
    ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
    ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
    uart_send_packet(&ack_packet);
    //вторая точка для калибровки
    current_duty = 50;
	pwm_set_duty(current_duty);
	#ifdef DEBUG_ENABLE
	uart_send_debug("waiting for cal_vout-_offset_voltage...");
	#endif
    while(!uart_async_has_packet()){}
	if (uart_async_get_packet(&rx_packet)){
		if (rx_packet.status & EDIT_VOUT_CALIBRATION_COMMAND) {//изменение калибровочного коэффицента VOUT
			y2 = (float)rx_packet.setpoint / 100.0f;
            for (uint8_t i = 0; i < 50; i++){
                x2 = get_output_adc_channel();
            }


		}else{
            error_flags |= UART_ERROR;
            #ifdef DEBUG_ENABLE
            uart_send_debug("error: cal_packet's been lost");
            #endif
            nack_packet.start = 0xAA;
            nack_packet.master_addr = rx_packet.master_addr;
            nack_packet.slave_addr = flash_get_slave_addr();
            nack_packet.packet_num = tx_packet_counter++;
            nack_packet.voltage = current_voltage; 
            nack_packet.setpoint = rx_packet.setpoint; 
            nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
            uart_send_packet(&nack_packet);
            return;
		}
	}else{
        error_flags |= UART_ERROR;
        #ifdef DEBUG_ENABLE
        uart_send_debug("error: cal_packet's been lost");
        #endif
        nack_packet.start = 0xAA;
        nack_packet.master_addr = rx_packet.master_addr;
        nack_packet.slave_addr = flash_get_slave_addr();
        nack_packet.packet_num = tx_packet_counter++;
        nack_packet.voltage = current_voltage; 
        nack_packet.setpoint = rx_packet.setpoint; 
        nack_packet.status = (NACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | (1 << NACK_WRONG_COMMAND);
        uart_send_packet(&nack_packet);
        return;
	}
    current_cal_out_voltage_value = (y2-y1)/(float)(x2-x1);
    current_cal_out_voltage_offset_value = y1 - current_cal_out_voltage_value * x1;

    flash_set_cal_out_voltage_value(current_cal_out_voltage_value);
    flash_set_cal_out_voltage_offset_value(current_cal_out_voltage_offset_value);
    update_calibration_values();
    
    #ifdef DEBUG_ENABLE
    uart_send_debug("VOUT calibration saved");
    uart_send_debug("Current vout coef(y =a1*x + a0):");
    uart_send_string(" a1: ");
    uart_send_debug_float(flash_get_cal_out_voltage_value());
    uart_send_string(" a0: ");
    uart_send_debug_float(flash_get_cal_out_voltage_offset_value());
    uart_send_debug(" V/count;");
    #endif

    ack_packet.start = 0xAA;
    ack_packet.master_addr = rx_packet.master_addr;
    ack_packet.slave_addr = flash_get_slave_addr();
    ack_packet.packet_num = tx_packet_counter++;
    ack_packet.voltage = current_voltage; // текущее измерение
    ack_packet.setpoint = rx_packet.setpoint; // подтверждаем полученную уставку
    ack_packet.status = (ACK_FLAG << PACKET_TYPE) | (system_mode << MODE_BIT) | error_flags;
    uart_send_packet(&ack_packet);

}

// ==================== ИЗМEРЕНИЯ ====================

static void update_measurements(void) {
    // Измеряем текущее напряжение
    current_voltage = get_output_voltage();
    
    // Проверка превышения напряжения
    if (current_voltage > (float) VOLTAGE_MAX+20) {
        error_flags |= (1 << OVERVOLTAGE_ERROR);  // Превышение напряжения
        // Аварийное отключение выхода
        current_duty = 0;
        pwm_set_duty(current_duty);
        #ifdef DEBUG_ENABLE
        uart_send_debug("ERROR: Overvoltage detected! PWM disabled\r\n");
        #endif
    } else if (current_voltage < (float) VOLTAGE_MIN-5) {
        error_flags |= (1 << UNDERVOLTAGE_ERROR);
    }
    else{
        error_flags &= ~(1 << OVERVOLTAGE_ERROR);
        error_flags &= ~(1 << UNDERVOLTAGE_ERROR);
    }

    if ((fabs(current_voltage_setpoint - current_voltage))/(float) current_voltage_setpoint > 0.05 ){
        error_flags |= (1 << MODE_ENTRY_TIMEOUT_ERROR);
    }
    else{
        error_flags &= ~(1 << MODE_ENTRY_TIMEOUT_ERROR);
    }
}


// ==================== ОТПРАВКА ПЕРИОДИЧЕСКИХ ДАННЫХ ====================
static void update_flash(){
    if (flash_get_setpoint() != current_voltage_setpoint) {
        flash_set_setpoint(current_voltage_setpoint);
    }
    if (flash_get_mode() != system_mode) {
        flash_set_mode(system_mode);
    }
    if (flash_get_status() != error_flags) {
        flash_set_status(error_flags);
    }
	if (fabs(flash_get_cal_adc_voltage_value() - current_cal_adc_voltage_value) > 0.0001f){
		flash_set_cal_adc_voltage_value(current_cal_adc_voltage_value);
	}
	if (fabs(flash_get_cal_out_voltage_value() - current_cal_out_voltage_value) > 0.0001f){
		flash_set_cal_out_voltage_value(current_cal_out_voltage_value);
	}
    if (fabs(flash_get_cal_out_voltage_offset_value() - current_cal_out_voltage_offset_value) > 0.0001f){
        flash_set_cal_out_voltage_offset_value(current_cal_out_voltage_offset_value);
    }
}

static void send_periodic_data(void) {
    tx_packet_t tx_packet;
    
    // Заполняем пакет
    tx_packet.start = 0xAA;
    tx_packet.master_addr = flash_get_master_addr();
    tx_packet.slave_addr = flash_get_slave_addr();
    tx_packet.packet_num = tx_packet_counter++;
    tx_packet.voltage = current_voltage;  // Округление до целых
    tx_packet.setpoint = current_voltage_setpoint;
    tx_packet.status = error_flags | ((system_mode ? 1 : 0) << 7);
    tx_packet.parity = 0;    
    // Отправляем пакет
    uart_send_packet(&tx_packet);
}
static void toggle_led(void) {
    LED_BUILTIN_PORT ^= (1 << LED_BUILTIN_PIN);
}
static uint16_t abs_diff(uint16_t a, int16_t b) {
    return (a > b) ? (a - b) : (b - a);
}
// ==================== ГЛАВНАЯ ФУНКЦИЯ ====================

int main(void) {
    uint16_t last_process_button_ms = 0;
    uint16_t last_tx_packet_time_ms = 0;
    uint16_t last_adc_measurement_time_ms = 0;
    uint16_t last_regulator_update_time_ms = 0;
    uint16_t current_time_ms = 0;
    // Инициализация всех модулей
    init_system();
    #ifdef DEBUG_ENABLE
    uart_send_debug("Main loop started (50Hz PI, 2Hz data send)\r\n\r\n");
    #endif

    while (1) {
        if (system_mode == MODE_WORK) {
            current_time_ms = get_time_ms();
            process_uart(); // обновляет уставку current_voltage_setpoint
            if (abs_diff(current_time_ms, last_process_button_ms) >= (float)PROCESS_BUTTONS_MS) {
                last_process_button_ms = current_time_ms;
                process_buttons(); // обновляет current_voltage_setpoint
            }
            if (abs_diff(current_time_ms, last_adc_measurement_time_ms) >= (float)MEASURE_VOLTAGE_MS) {
                last_adc_measurement_time_ms = current_time_ms;
                update_measurements(); // обновляет current_voltage и error_flags
            }
            if (abs_diff(current_time_ms, last_regulator_update_time_ms) >= (float) REGULATOR_UPDATE_MS) {
                last_regulator_update_time_ms = current_time_ms;
                update_control(); // обновляет current_duty на основе ПИ-регулятора
            }
            if (error_flags != 0){
                #ifdef DEBUG_ENABLE
                uart_send_string("Error flags updated: ");
                uart_send_debug_num(BIN, error_flags);
                uart_send_string("\r\n");
                #endif
            }
            update_flash();

        }
        else { //MODE_TEST
            current_time_ms = get_time_ms();
            process_uart(); // обновляет уставку current_voltage_setpoint
               
            if (abs_diff(current_time_ms, last_process_button_ms) >=(float) PROCESS_BUTTONS_MS) {
                last_process_button_ms = current_time_ms;
                process_buttons(); // обновляет current_duty
            }

            if (abs_diff(current_time_ms, last_adc_measurement_time_ms) >=(float) MEASURE_VOLTAGE_MS) {
                last_adc_measurement_time_ms = current_time_ms;
                update_measurements(); // обновляет current_voltage и error_flags
            }
            // Период отправки данных
            if (abs_diff(current_time_ms, last_tx_packet_time_ms) >= (float)PERIODIC_SEND_MS) {
                last_tx_packet_time_ms = current_time_ms;
                send_periodic_data();
                #ifdef DEBUG_ENABLE
                uart_send_tx_packet_debug((tx_packet_t){
                    .start = 0xAA,
                    .master_addr = flash_get_master_addr(),
                    .slave_addr = flash_get_slave_addr(),
                    .packet_num = tx_packet_counter,
                    .voltage = current_voltage,
                    .setpoint = current_voltage_setpoint,
                    .status = error_flags | ((system_mode ? 1 : 0) << 7),
                    .parity = 0
                });
                #endif
                //toggle_led(); // Мигаем светодиодом при каждой отправке
            }




            if (error_flags != 0){
                #ifdef DEBUG_ENABLE
                uart_send_string("Error flags updated: ");
                uart_send_debug_num(BIN, error_flags);
                uart_send_string("\r\n");
                #endif
            }
            update_flash();
        }
    }
    
    return 0;
}