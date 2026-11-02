// ==================== uart.c ====================
// Модуль UART для ATmega2560
// Протокол: 9600 8N1, асинхронный и синхронный приём пакетов

#include "uart.h"
#include "flash.h"

// ==================== СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ ====================

static volatile uint8_t current_slave_addr;
static volatile uint8_t current_master_addr;
// Буфер для асинхронного приёма (один пакет)
static volatile rx_packet_t async_buffer;
static volatile uint8_t new_packet_available = 0;
static volatile uint32_t packet_start_time = 0;
static volatile uint16_t lost_packet_counter = 0;
static volatile uint8_t packet_was_lost = 0;

// Парсинг входящего потока
static volatile uint8_t rx_buffer[RX_PACKET_LEN+5];             // Буфер для байт UART
static volatile uint8_t rx_index = 0;
static volatile  uint8_t expecting_packet = 0;      // 0 - ищем старт, 1 - собираем пакет
static volatile uint8_t async_enabled = 1;         // Флаг разрешения асинхронного приёма


// Вычисление CRC-16 для блока данных (простой алгоритм)
// Полином: 0x8005 (CRC-16-IBM)
static uint16_t calculate_parity(const uint8_t* data, uint8_t length) {
	uint16_t crc = 0xFFFF;
	
	for (uint8_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x8005;
				} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

static void toggle_led_async(void) {
	LED_BUILTIN_PORT ^= (1 << LED_BUILTIN_PIN);
}

void uart_set_slave_addr(uint8_t addr) {
    current_slave_addr = addr;
}
void uart_set_master_addr(uint8_t addr) {
    current_master_addr = addr;
}

void uart_send_string(const char* str) {
    while (*str) {
        uart_send_byte(*str++);
    }
}


void uart_init(uint8_t slave_addr, uint8_t master_addr) {
    current_slave_addr = slave_addr;
    current_master_addr = master_addr;

    // Настройка пинов TX/RX (UART0 использует PE0(RX) и PE1(TX))
    // Для Mega2560 UART0 - это порт E
    DDRE &= ~(1 << PE0);  // RX как вход
    DDRE |= (1 << PE1);   // TX как выход
    
    // Расчёт UBRR для 16 МГц
    // UBRR = F_CPU / (16 * BAUD) - 1
    uint16_t ubrr = (F_CPU / (16UL * BAUD_RATE)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    
    // Настройка формата: 8 бит данных, 1 стоп-бит, без чётности
    UCSR0C = 0;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 бит
    UCSR0B = (1 << TXEN0);   // Включаем TX и RX (без прерываний пока)
    UCSR0B |= (1 << RXEN0);
    // Сброс буферов парсинга
    rx_index = 0;
    expecting_packet = 0;
    new_packet_available = 0;
    async_enabled = 0;
    lost_packet_counter = 0;
    packet_was_lost = 0;
    
}
void uart_send_byte(uint8_t byte) {
    while (!(UCSR0A & (1 << UDRE0)));  // Ждём, пока буфер передачи пуст
    UDR0 = byte;
}
void uart_send_bytes(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uart_send_byte(data[i]);
    }
}


void uart_send_settings_debug(void) {
    uart_send_debug("Current Settings: ");
    uart_send_string("Version: ");
    uart_send_debug_num(DEC, (uint16_t) flash_get_version());
    uart_send_string("\nSetpoint: ");
    uart_send_debug_num(DEC, flash_get_setpoint());
    uart_send_string("\nSlave Addr: ");
    uart_send_debug_num(HEX, (uint16_t)flash_get_slave_addr());
    uart_send_string(", Master Addr: ");
    uart_send_debug_num(HEX, (uint16_t)flash_get_master_addr());
    uart_send_string("\nMode: ");
    uint8_t mode = flash_get_mode();
    uart_send_debug_num(DEC, (uint16_t)mode);
    if (mode == 0) {
        uart_send_string(" (Test)");
    } else{
        uart_send_string(" (Work)");
    }
    uart_send_string("\nCal ADC Voltage: ");
    uart_send_debug_float(flash_get_cal_adc_voltage_value());
    uart_send_string(" V/count");
    uart_send_string("\nCal OUT Voltage: ");
    uart_send_debug_float(flash_get_cal_out_voltage_value());
    uart_send_string(" V/count");
    uart_send_string("\nCal OUT Voltage Offset: ");
    uart_send_debug_float(flash_get_cal_out_voltage_offset_value());
    uart_send_string(" V/count");
    uart_send_string("\nStatus: ");
    uint8_t status = flash_get_status();
    uart_send_debug_num(BIN, (uint16_t)status);
    uart_send_byte(' ');
    uart_send_debug_num(HEX, (uint16_t)status);
    uart_send_byte('\r');
    uart_send_byte('\n');
}
void uart_send_debug(const char* str) {
    while (*str) {
        uart_send_byte(*str++);
    }
    uart_send_byte('\r');
    uart_send_byte('\n');
}
void uart_send_debug_num(uint8_t format, uint16_t num) {
    switch (format) {
        case BIN: {
            uart_send_string("0b");
            // Преобразуем число в двоичную строку
            for (int8_t i = 15; i >= 0; i--) {
                uart_send_byte((num & (1 << i)) ? '1' : '0');
            }
            break;
        }
        case HEX: {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "0x%X", num); // Шестнадцатеричный формат
            uart_send_string(buffer);
            break;
        }
        case DEC: {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%u", num); // Десятичный формат
            uart_send_string(buffer);
            break;
        }
        default:
            break;
    }
}
void uart_send_debug_float(float num) {
	char buffer[16];
	dtostrf(num, 6, 3, buffer);  // ширина 6, 3 знака после запятой
	uart_send_string(buffer);
}

void uart_send_tx_packet_debug(tx_packet_t packet) {
    uint16_t parity_value = calculate_parity((const uint8_t *)&packet, sizeof(tx_packet_t) - 2); // Вычисляем контрольную сумму для отладки
    uart_send_string("\n\rPacket: ");
    uart_send_byte('[');
    uart_send_debug_num(HEX, (uint16_t) packet.start);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) packet.master_addr);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) packet.slave_addr);
    uart_send_byte(',');
    uart_send_debug_num(DEC, (uint16_t) packet.packet_num);
    uart_send_byte(',');
    uart_send_debug_num(DEC, (uint16_t)packet.voltage);
    uart_send_string("V(OUT), ");
    uart_send_debug_num(DEC, packet.setpoint);
    uart_send_string("V(SETPOINT), ");
    uart_send_debug_num(BIN, (uint16_t) packet.status);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) parity_value);
    uart_send_byte(']');
    uart_send_byte('\r');
    uart_send_byte('\n');
}
void uart_send_rx_packet_debug(rx_packet_t packet) {
    //uint16_t parity_value = calculate_parity(&packet, sizeof(rx_packet_t) - 1); // Вычисляем контрольную сумму для отладки
    uart_send_string("Received Packet: ");
    uart_send_byte('[');
    uart_send_debug_num(HEX, (uint16_t) packet.start);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) packet.master_addr);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) packet.slave_addr);
    uart_send_byte(',');
    uart_send_debug_num(DEC, (uint16_t) packet.packet_num);
    uart_send_byte(',');
    uart_send_debug_num(DEC, packet.setpoint);
    uart_send_string("V(SETPOINT), ");
    uart_send_debug_num(BIN, (uint16_t) packet.status);
    uart_send_byte(',');
    uart_send_debug_num(HEX, (uint16_t) packet.parity);
    uart_send_byte(']');
    uart_send_byte('\r');
    uart_send_byte('\n');
}

// Отправка пакета данных (slave -> master)
void uart_send_packet(const tx_packet_t* packet) {
    uint8_t tx_buffer[sizeof(tx_packet_t)];
    uint8_t len = 0;
    uint8_t float_bytes[4];
    memcpy((void *)float_bytes, (const void *)&packet->voltage, sizeof(float));//от младшего к старшему байту
    // Формируем буфер для отправки
    tx_buffer[len++] = packet->start;
    tx_buffer[len++] = packet->master_addr;
    tx_buffer[len++] = packet->slave_addr;
    tx_buffer[len++] = packet->packet_num;
    tx_buffer[len++] = float_bytes[0];  // малдший байт
    tx_buffer[len++] = float_bytes[1]; 
    tx_buffer[len++] = float_bytes[2];
    tx_buffer[len++] = float_bytes[3]; // старший байт
    tx_buffer[len++] = (uint8_t)(packet->setpoint >> 8); // старший байт уставки
    tx_buffer[len++] = (uint8_t)(packet->setpoint & 0xFF); // младший байт уставки
    tx_buffer[len++] = packet->status;
    uint16_t parity = calculate_parity(tx_buffer, len);
    tx_buffer[len++] = (uint8_t)(parity >> 8);
    tx_buffer[len++] = (uint8_t)(parity & 0xFF);
    
    // Отправляем
    uart_send_bytes(tx_buffer, len);
}


// Включение асинхронного приёма (прерывания)
void uart_async_enable(void) {
    async_enabled = 1;
    UCSR0B |= (1 << RXCIE0);   // Включаем прерывание по приёму
    rx_index = 0;
    expecting_packet = 0;
    new_packet_available = 0;
}
// Отключение асинхронного приёма
void uart_async_disable(void) {
    async_enabled = 0;
    UCSR0B &= ~(1 << RXCIE0);  // Выключаем прерывание
}


uint8_t uart_async_has_packet(void) {
    return new_packet_available;
}
uint8_t uart_async_get_packet(rx_packet_t* packet) {
    if (!new_packet_available) {
        return 0;
    }
	cli();
    // Копируем данные из буфера
    packet->start = async_buffer.start;
    packet->master_addr = async_buffer.master_addr;
    packet->slave_addr = async_buffer.slave_addr;
    packet->packet_num = async_buffer.packet_num;
    packet->setpoint = async_buffer.setpoint;
    packet->status = async_buffer.status;
    packet->parity = async_buffer.parity;
    new_packet_available = 0;
	sei();
    return 1;
}

uint8_t uart_async_was_packet_lost(void) {
    if (packet_was_lost) {
        packet_was_lost = 0; // Сброс флага после проверки
        return 1;
    }
    return 0;
}
uint16_t uart_async_get_lost_count(void) {
    return lost_packet_counter;
}

// ==================== ПРЕРЫВАНИЕ UART RX ====================
ISR(USART0_RX_vect) {
	//uart_send_byte('!');
    if (!async_enabled) {
        // Если асинхронный приём отключён - читаем и игнорируем байт
        (void)UDR0;
        return;
    }
    uint8_t byte = UDR0;
    if (!expecting_packet) {//сюда если expecting_packet = 0
        // Ищем стартовый байт 0x55
        if (byte == 0x55) {
			//toggle_led_async();
			//uart_send_debug("Start!!!");
            rx_buffer[0] = byte;
            rx_index = 1;
            expecting_packet = 1;
        }
    } else {//сюда если expecting_packet = 1
        rx_buffer[rx_index++] = byte;
       if (rx_index >= RX_PACKET_LEN) {  // 0x55 + master + slave + num + setpoint_H + setpoint_L + status + checksum = 9 байт
            expecting_packet = 0;
            if (rx_buffer[2] == current_slave_addr && !new_packet_available) {  // slave_addr на позиции 2
                if (calculate_parity(rx_buffer, RX_PACKET_LEN - 2) == ((rx_buffer[RX_PACKET_LEN - 2] << 8) | rx_buffer[RX_PACKET_LEN - 1])) {  // Проверка контрольной суммы
                    // Контрольная сумма верна - сохраняем пакет в асинхронный буфер
                    //uart_send_debug("good!!!");
					async_buffer.start = rx_buffer[0];
                    async_buffer.master_addr = rx_buffer[1];
                    async_buffer.slave_addr = rx_buffer[2];
                    async_buffer.packet_num = rx_buffer[3];
                    async_buffer.setpoint = (rx_buffer[4] << 8) | rx_buffer[5];
                    async_buffer.status = rx_buffer[6];
                    async_buffer.parity = (rx_buffer[7] << 8) | rx_buffer[8];
                    new_packet_available = 1;
                }else{
                    packet_was_lost = 1;
                    lost_packet_counter++;
                }
            }
            
            rx_index = 0;
        }
        // Защита от переполнения буфера
        if (rx_index >= sizeof(rx_buffer)) {
            rx_index = 0;
            expecting_packet = 0;
        }
    }
}