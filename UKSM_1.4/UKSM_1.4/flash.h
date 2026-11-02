// ==================== flash.h ====================
#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include "config.h"
// ==================== НАСТРОЙКИ ПО УМОЛЧАНИЮ ====================
// Можно переопределить в main.c перед #include "flash.h"


// Адреса в EEPROM (ATmega2560 имеет 4KB EEPROM)
// Используем начало памяти для хранения настроек
#define EEPROM_ADDR_SETTINGS    0x00   // Начальный адрес структуры настроек

// ==================== СТРУКТУРА НАСТРОЕК ====================
typedef struct {
    uint8_t version;
    uint16_t setpoint;          // Текущая уставка напряжения (30-250В)
    uint8_t slave_addr;         // Адрес этого устройства
    uint8_t master_addr;        // Адрес компьютера (master)
    uint8_t mode;               // 0 - тестовый, 1 - рабочий
    float cal_adc_voltage_value; // Калибровочный коэффициент для АЦП (напряжение на входе)
    float cal_out_voltage_value; // Калибровочный коэффициент для выходного напряж
    float cal_out_voltage_offset_value; // Калибровочный коэффициент для выходного напряжения (смещение)
    uint8_t reserved[3];        // Зарезервировано для будущего расширения
    uint8_t status;
    uint16_t crc;               // Контрольная сумма (CRC-16)

} settings_t;

// ==================== ФУНКЦИИ ====================

// Инициализация EEPROM и загрузка настроек
// Возвращает: 1 - загружены сохранённые настройки, 0 - использованы настройки по умолчанию
uint8_t flash_init(void);

// Сохранение текущих настроек в EEPROM
void flash_save(void);

// Сброс настроек к значениям по умолчанию
void flash_reset_to_defaults(void);

// ==================== ГЕТТЕРЫ ====================
uint8_t flash_get_version(void);
uint16_t flash_get_setpoint(void);
uint8_t  flash_get_slave_addr(void);
uint8_t  flash_get_master_addr(void);
uint8_t  flash_get_mode(void);
uint8_t flash_get_status(void);
float flash_get_cal_adc_voltage_value(void);
float flash_get_cal_out_voltage_value(void);
float flash_get_cal_out_voltage_offset_value(void);
// ==================== СЕТТЕРЫ ====================
void flash_set_version(uint8_t value);
void flash_set_setpoint(uint16_t value);
void flash_set_slave_addr(uint8_t addr);
void flash_set_master_addr(uint8_t addr);
void flash_set_mode(uint8_t mode);
void flash_set_status(uint8_t status);
void flash_set_cal_adc_voltage_value(float value);
void flash_set_cal_out_voltage_value(float value);
void flash_set_cal_out_voltage_offset_value(float value);
// ==================== ВСПОМОГАТЕЛЬНЫЕ ====================
// Получение указателя на структуру настроек (для прямого доступа)
settings_t* flash_get_settings(void);

#endif // FLASH_H