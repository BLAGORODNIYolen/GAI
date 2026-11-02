// ==================== input.h ====================
#ifndef INPUT_H
#define INPUT_H
#include "config.h"
#include <stdint.h>


// ==================== НАСТРОЙКИ ПО УМОЛЧАНИЮ ====================
// Можно переопределить в main.c перед #include "input.h"

#ifndef BTN_UP_PIN
#define BTN_UP_PIN     PD0   // Кнопка UP (пин 21 на Mega)
#endif

#ifndef BTN_DOWN_PIN
#define BTN_DOWN_PIN   PD1   // Кнопка DOWN (пин 20 на Mega)
#endif

#ifndef BTN_PORT
#define BTN_PORT       PORTD
#endif

#ifndef BTN_PIN_REG
#define BTN_PIN_REG    PIND
#endif

#ifndef BTN_DDR_REG
#define BTN_DDR_REG    DDRD
#endif

// Пины АЦП
#ifndef ADC_VOLTAGE_CH
#define ADC_VOLTAGE_CH 0     // Канал АЦП для измерения выходного напряжения (A0)
#endif

#ifndef ADC_POT_CH
#define ADC_POT_CH     1     // Канал АЦП для потенциометра (A1)
#endif

// Коэффициенты преобразования АЦП в напряжение (30-250В)
// При опорном AVcc=5В и разрешении 10 бит (0-1023)
// Напряжение = raw * (250.0 / 1023.0) но с учётом делителя
// Для делителя, например, 1:50 (250В -> 5В на входе АЦП)
#ifndef VOLTAGE_MIN
#define VOLTAGE_MIN     30
#endif

#ifndef VOLTAGE_MAX
#define VOLTAGE_MAX     250
#endif

#define DEBOUNCE_PRESSED_CONDITION 9 
#define DEBOUNCE_RELEASED_CONDITION 7
#define DEBOUNCE_MAX_COUNTER 15

#define ADC_FILTER_SIZE 12
// ==================== ФУНКЦИИ ====================

// Инициализация кнопок (настраивает пины как входы с подтяжкой)
void buttons_init(void);

// Инициализация АЦП
void adc_init(void);
void update_calibration_values(void);





uint16_t adc_read_channel(uint8_t channel);//функция -драйвер кода АЦП

uint16_t get_output_adc_channel(void);//сырой код АЦП на канале VOUT
float get_channel_voltage(uint8_t channel);//напряжение на входе АЦП на канале VOUT


float get_output_voltage(void); //выходное напряжение
uint16_t get_pot_setpoint(void);



// Чтение состояния кнопок с антидребезгом
// Возвращает 1 если кнопка нажата, иначе 0
uint8_t if_button_up_was_pressed(void);
uint8_t if_button_down_was_pressed(void);
// Чтение выходного напряжения (сырое значение АЦП)


#endif // INPUT_H