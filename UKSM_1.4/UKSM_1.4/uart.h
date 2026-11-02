// ==================== uart.h ====================
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"


#define RX_PACKET_LEN 9
#define TX_PACKET_LEN 13
#define PACKET_TIMEOUT_MS 50  // Макс. время сбора пакета





typedef enum {
    BIN,
    HEX,
    DEC,
    FLOAT
} num_formats;
// ==================== СТРУКТУРЫ ПАКЕТОВ ====================

// Пакет отправки (slave -> master)
typedef struct {
    uint8_t start;          // 0xAA
    uint8_t master_addr;    // Адрес мастера (компьютер)
    uint8_t slave_addr;     // Адрес этого устройства
    uint8_t packet_num;     // Номер пакета (инкрементируется)
    float voltage;       // Измеренное напряжение (0-250В, целые)
    uint16_t setpoint;      // Текущая уставка (0-250В)
    uint8_t status;         // Бит 0: ошибка UART, бит 1: перегрев, бит 2: превышение напряжения и т.д.
    uint16_t parity;         // Контрольная сумма (CRC-16)
} tx_packet_t;

// Пакет приёма (master -> slave)
typedef struct {
    uint8_t start;          // 0x55
    uint8_t master_addr;    // Адрес мастера
    uint8_t slave_addr;     // Адрес получателя
    uint8_t packet_num;     // Номер пакета (для контроля целостности)
    uint16_t setpoint;      // Новая уставка (0-250В)
    uint8_t status;  // Команды: бит 0 = сброс, бит 1 = переключить режим и т.д.
    uint16_t parity;         
} rx_packet_t;

// ==================== ФУНКЦИИ ====================


// Инициализация UART
void uart_init(uint8_t slave_addr, uint8_t master_addr);
void uart_set_slave_addr(uint8_t addr);
void uart_set_master_addr(uint8_t addr);
void uart_send_byte(uint8_t byte);
void uart_send_bytes(const uint8_t* data, uint8_t len);
void uart_send_string(const char* str);


// Отправка дебаг
void uart_send_settings_debug(void);
void uart_send_debug(const char* str);
void uart_send_debug_num(uint8_t format, uint16_t num);
void uart_send_debug_float(float num);
void uart_send_tx_packet_debug(tx_packet_t packet);
void uart_send_rx_packet_debug(rx_packet_t packet);
// Отправка пакета данных

void uart_send_packet(const tx_packet_t* packet);
// Управление асинхронным приёмом
void uart_async_enable(void);
void uart_async_disable(void);

// Получение пакета из асинхронного буфера
uint8_t uart_async_has_packet(void);
uint8_t uart_async_get_packet(rx_packet_t* packet);

uint8_t uart_async_was_packet_lost(void);
uint16_t uart_async_get_lost_count(void);
#endif // UART_H