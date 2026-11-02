// ==================== input.c ====================
#include "input.h"
#include "flash.h"
#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Для чтения напряжения с АЦП
//void adc_read_channel(
//channel: ADC_VOLTAGE_CH - напряжение
//channel: ADC_POT_CH - потенциометр
//)

// =============== Глобальные переменные ================
//Калибровочные коэффициенты получаются из флэш-памяти при инициализации АЦП
static float cal_adc_voltage_value_copy = 0.0f; // Коэффициент для преобразования raw АЦП в напряжение на входе
static float cal_out_voltage_value_copy = 0.0f; // Коэффициент для преобраз  ования raw АЦП в выходное напряжение (с учётом делителя)
static float cal_out_voltage_offset_value_copy = 0.0f; // Смещение для выходного напряжения (на случай, если есть постоянная ошибка)
// ==================== СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ ====================
// Для антидребезга кнопок
static uint8_t up_button_state = 1;      
static uint8_t down_button_state = 1;
static uint8_t up_debounce_counter = 0;
static uint8_t down_debounce_counter = 0;
static uint8_t up_current = 1; //текущее состояние кнопки UP (1 - отпущена, 0 - нажата)
static uint8_t down_current = 1; //текущее состояние кнопки DOWN (
static uint8_t up_previous_state = 1; //предыдущее состояние кнопки UP
static uint8_t down_previous_state = 1; //предыдущее состояние кнопки DOWN
static uint8_t up_pressed_event = 0; //флаг события нажатия кнопки UP
static uint8_t down_pressed_event = 0; //флаг события нажатия кнопки DOWN

static uint16_t adc_buffer[ADC_FILTER_SIZE];
static uint8_t adc_index = 0;
static uint32_t adc_sum = 0;

void update_calibration_values(void) {
    cal_adc_voltage_value_copy = flash_get_cal_adc_voltage_value();
    cal_out_voltage_value_copy = flash_get_cal_out_voltage_value();
    cal_out_voltage_offset_value_copy = flash_get_cal_out_voltage_offset_value();
}



// ==================== ИНИЦИАЛИЗАЦИЯ ====================

void buttons_init(void) {
    // Настройка пинов кнопок как входов
    BTN_DDR_REG &= ~((1 << BTN_UP_PIN) | (1 << BTN_DOWN_PIN)); 
    // Включение внутренних подтягивающих резисторов (пин подтянут к VCC)
    BTN_PORT |= (1 << BTN_UP_PIN) | (1 << BTN_DOWN_PIN);
}

void adc_init(void) {
    // Настройка АЦП
    // Опорное напряжение: AVcc (5В)
    ADMUX = (1 << REFS0);           // REFS0=1, REFS1=0 -> AVcc с внешним конденсатором на AREF
    
    // Включаем АЦП, предделитель = 128 (частота АЦП ~125 кГц при 16 МГц)
    // Это даёт хорошее качество преобразования
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // Выполняем несколько "пустых" преобразований для стабилизации АЦП
    for (uint8_t i = 0; i < 10; i++) {
        ADCSRA |= (1 << ADSC);         // Запуск преобразования
        while (ADCSRA & (1 << ADSC)); // Ожидание завершения
        (void)ADCW;                  // Чтение результата (игнорируем)
    }
    update_calibration_values();
	for (uint8_t i = 0; i < ADC_FILTER_SIZE; i++){
		adc_buffer[i] =0;
	}
	for (uint8_t i = 0; i < ADC_FILTER_SIZE; i++){
		get_output_voltage();
	}

}

// ==================== Функции для вызова из main====================

// Чтение АЦП с указанного канала (0-15)
uint16_t adc_read_channel(uint8_t channel) {
    // Выбор канала (0-7 - обычные, 8-15 - через мультиплексор)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Запуск преобразования
    ADCSRA |= (1 << ADSC);
    // Ожидание завершения
    while (ADCSRA & (1 << ADSC));
    // Возврат результата
    return ADCW;
}
float get_channel_voltage(uint8_t channel) {
    float result = 0.0f;
    uint16_t raw = adc_read_channel(channel);
    result = cal_adc_voltage_value_copy*(float)raw; // Преобразование в напряжение на входе АЦП
    return result; // Для других каналов возвращаем 0.0f - не обрабатываемые
}
uint16_t get_output_adc_channel(void){
	uint16_t raw = adc_read_channel(ADC_VOLTAGE_CH);
	// Скользящее среднее
	adc_sum = adc_sum - adc_buffer[adc_index] + raw;
	adc_buffer[adc_index] = raw;
	adc_index = (adc_index + 1) % ADC_FILTER_SIZE;
	uint16_t filtered_raw = adc_sum / ADC_FILTER_SIZE;
	return filtered_raw;
}
float get_output_voltage(void) {
	float result = cal_out_voltage_value_copy * (float)get_output_adc_channel() + cal_out_voltage_offset_value_copy; // Преобразование в выходное напряжение с учётом калибровки
	return result;
}

uint16_t get_pot_setpoint(void) { //уставка 30-250В
    uint16_t raw = adc_read_channel(ADC_POT_CH);
    uint32_t range = VOLTAGE_MAX - VOLTAGE_MIN; // 250-30 = 220
    uint16_t setpoint = VOLTAGE_MIN + (uint16_t)((range * (uint32_t)raw) / 1023);
    return setpoint;
}

// ==================== ЧТЕНИЕ КНОПОК С АНТИДРЕБЕЗГОМ ====================

// Функция должна вызываться достаточно часто (каждые 1-5 мс)
// Рекомендуется вызывать из прерывания таймера или в главном цикле с задержкой

static void update_buttons_state(void) {
    //1 -отпущена, 0 - нажата
    up_current = (BTN_PIN_REG & (1 << BTN_UP_PIN));
    down_current = (BTN_PIN_REG & (1 << BTN_DOWN_PIN));
    if (!up_current){//если нажата кнопка UP
        if (up_debounce_counter < DEBOUNCE_MAX_COUNTER) {
            up_debounce_counter++;
        }
    } else {
        if (up_debounce_counter > 0) {
            up_debounce_counter--;
        }
    }
    if (up_debounce_counter >= DEBOUNCE_PRESSED_CONDITION) {
        up_button_state = 0; //кнопка считается нажатой
    } else if (up_debounce_counter <= DEBOUNCE_RELEASED_CONDITION) {
        up_button_state = 1; //кнопка считается отпущенной
    }
    if (up_button_state != up_previous_state){
        if (up_button_state == 0){ //кнопка UP была нажата
            up_pressed_event = 1; //устанавливаем флаг события нажатия
        }
        up_previous_state = up_button_state; //обновляем предыдущее состояние
    }






    if (!down_current){//если нажата кнопка DOWN
        if (down_debounce_counter < DEBOUNCE_MAX_COUNTER) {
            down_debounce_counter++;
        }
    } else {
        if (down_debounce_counter > 0) {
            down_debounce_counter--;
        }
    }
    if (down_debounce_counter >= DEBOUNCE_PRESSED_CONDITION) {
        down_button_state = 0; //кнопка считается нажатой
    } else if (down_debounce_counter <= DEBOUNCE_RELEASED_CONDITION) {
        down_button_state = 1; //кнопка считается отпущенной
    }
    if (down_button_state != down_previous_state){
        if (down_button_state == 0){ //кнопка DOWN была нажата
            down_pressed_event = 1; //устанавливаем флаг события нажатия
        }
        down_previous_state = down_button_state; //обновляем предыдущее состояние
    }
}

// Возвращает 1 если кнопка UP нажата (0 - нажата, т.к. подтяжка к питанию)
uint8_t if_button_up_was_pressed(void) {
    update_buttons_state();
    if (up_pressed_event) {
        up_pressed_event = 0; //сбрасываем флаг события
        return 1; //сообщаем о событии нажатия
    }
    return 0;
}

uint8_t if_button_down_was_pressed(void) {
    update_buttons_state();
    if (down_pressed_event) {
        down_pressed_event = 0; //сбрасываем флаг события
        return 1; //сообщаем о событии нажатия
    }
    return 0;
}
