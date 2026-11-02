// ==================== flash.c ====================
#include "flash.h"
#include <avr/eeprom.h>

// ==================== СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ ====================
// Кэш настроек в ОЗУ (для быстрого доступа и уменьшения числа операций с EEPROM)
static settings_t current_settings;
// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
// Вычисление CRC-16 для блока данных (простой алгоритм)
// Полином: 0x8005 (CRC-16-IBM)
static uint16_t calculate_crc(const uint8_t* data, uint8_t length) {
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
// Вычисление CRC для структуры настроек (без поля crc)
static uint16_t calculate_settings_crc(const settings_t* settings) {
	// Вычисляем CRC для всех байт, кроме последних двух (поле crc)
	uint8_t* data = (uint8_t*)settings;
	uint8_t length = sizeof(settings_t) - sizeof(uint16_t);
	return calculate_crc(data, length);
}



// Сохранение текущих настроек в EEPROM
void flash_save(void) {
    // Пересчитываем CRC перед сохранением
    current_settings.crc = calculate_settings_crc(&current_settings);
    // Записываем в EEPROM
    eeprom_update_block(&current_settings, (void*)EEPROM_ADDR_SETTINGS, sizeof(settings_t));
}




// Проверка валидности настроек по CRC
static uint8_t is_settings_valid(const settings_t* settings) {
    uint16_t expected_crc = calculate_settings_crc(settings);
    return (expected_crc == settings->crc);
}

// Установка настроек по умолчанию
static void set_default_settings(void) {
    current_settings.version = DEFAULT_VERSION;  // Версия структуры настроек
    current_settings.setpoint = DEFAULT_SETPOINT;
    current_settings.slave_addr = SLAVE_ADDRESS;
    current_settings.master_addr = MASTER_ADDRESS;
    current_settings.mode = DEFAULT_MODE;
    current_settings.status = DEFAULT_STATUS;
    // Обнуляем резервные байты
    current_settings.reserved[0] = 0;
    current_settings.reserved[1] = 0;
    current_settings.reserved[2] = 0;
    current_settings.cal_adc_voltage_value = DEFAULT_CAL_ADC_VOLTAGE_VALUE; // Считанное значение АЦП при калибровке
    current_settings.cal_out_voltage_value = DEFAULT_CAL_OUT_VOLTAGE_VALUE; // Измеренное напряжение при калибровке
    current_settings.cal_out_voltage_offset_value = DEFAULT_CAL_OUT_VOLTAGE_OFFSET_VALUE; // Смещение выходного напряжения при калибровке
    
    // Вычисляем и устанавливаем CRC
    current_settings.crc = calculate_settings_crc(&current_settings);
    flash_save();
}

// ==================== ОСНОВНЫЕ ФУНКЦИИ ====================

// Инициализация: загрузка настроек из EEPROM или установка по умолчанию
uint8_t flash_init(void) {
    // Читаем настройки из EEPROM в кэш
    eeprom_read_block(&current_settings, (const void*)EEPROM_ADDR_SETTINGS, sizeof(settings_t));
    
    // Проверяем валидность CRC
    if (is_settings_valid(&current_settings)) {
        // Настройки валидны, используем их
        return 1;  // Загружены сохранённые настройки
    } else {

        // CRC не совпадает - настройки повреждены или EEPROM не инициализирована
        set_default_settings();

        return 0;  // Использованы настройки по умолчанию
    }
}


// Сброс к значениям по умолчанию
void flash_reset_to_defaults(void) {
    set_default_settings();
}

// ==================== ГЕТТЕРЫ ====================
uint8_t flash_get_version(void) {
    return current_settings.version;
}
uint16_t flash_get_setpoint(void) {
    return current_settings.setpoint;
}

uint8_t flash_get_slave_addr(void) {
    return current_settings.slave_addr;
}

uint8_t flash_get_master_addr(void) {
    return current_settings.master_addr;
}

uint8_t flash_get_mode(void) {
    return current_settings.mode;
}
uint8_t flash_get_status(void) {
    return current_settings.status;
}
float flash_get_cal_adc_voltage_value(void) {
    return current_settings.cal_adc_voltage_value;
}
float flash_get_cal_out_voltage_value(void) {
    return current_settings.cal_out_voltage_value;
}
float flash_get_cal_out_voltage_offset_value(void) {
    return current_settings.cal_out_voltage_offset_value;
}
// ==================== СЕТТЕРЫ ====================
void flash_set_version(uint8_t value){
    current_settings.version = value;
    flash_save(); // Сохраняем изменения в EEPROM
}
void flash_set_setpoint(uint16_t value) {
    // Ограничиваем значение допустимым диапазоном (30-250В)
    if (value < 30) value = 30;
    if (value > 250) value = 250;
    
    current_settings.setpoint = value;
    flash_save(); // Сохраняем изменения в EEPROM
}

void flash_set_slave_addr(uint8_t addr) {
    current_settings.slave_addr = addr;
    flash_save(); // Сохраняем изменения в EEPROM
}

void flash_set_master_addr(uint8_t addr) {
    current_settings.master_addr = addr;
    flash_save(); // Сохраняем изменения в EEPROM
}

void flash_set_mode(uint8_t mode) {
    // Только 0 или 1
    if (mode >= 1) mode = 1;
    current_settings.mode = mode;
    flash_save(); // Сохраняем изменения в EEPROM
}
void flash_set_status(uint8_t status) {
    current_settings.status = status;
    flash_save(); // Сохраняем изменения в EEPROM
}
void flash_set_cal_adc_voltage_value(float value) {
    current_settings.cal_adc_voltage_value = value;
    flash_save(); // Сохраняем изменения в EEPROM
}
void flash_set_cal_out_voltage_value(float value) {
    current_settings.cal_out_voltage_value = value;
    flash_save(); // Сохраняем изменения в EEPROM
}
void flash_set_cal_out_voltage_offset_value(float value) {
    current_settings.cal_out_voltage_offset_value = value;
    flash_save(); // Сохраняем изменения в EEPROM
}
// ==================== ДОПОЛНИТЕЛЬНЫЕ ФУНКЦИИ ====================

// Получение указателя на структуру настроек (для прямого доступа)
settings_t* flash_get_settings(void) {
    return &current_settings;
}