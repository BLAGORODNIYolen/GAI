// ==================== pwm.h ====================
#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "config.h"
// ==================== НАСТРОЙКИ ПО УМОЛЧАНИЮ ====================
// Можно переопределить в main.c перед #include "pwm.h"

#ifndef PWM_PIN
#define PWM_PIN         PB5   // OC1A (пин 11 на Mega)
#endif

#ifndef PWM_FREQUENCY_HZ
#define PWM_FREQUENCY_HZ 5000  
#endif

#ifndef PWM_MAX_DUTY
#define PWM_MAX_DUTY    100    // Максимальная скважность (0-1000 = 0-100%)
#endif

#ifndef PWM_MAX_AVAILABLE_DUTY
#define PWM_MAX_AVAILABLE_DUTY 90  
#endif // Максимальная доступная скважность (для защиты катушки при старте)
#ifndef PWM_TOP
#define PWM_TOP    399    // Максимальная скважность (0-1000 = 0-100%)
#endif

// Параметры ПИ-регулятора по умолчанию
#ifndef PI_KP
#define PI_KP           0.2f//0.1f   // Пропорциональный коэффициент
#endif

#ifndef PI_KI
#define PI_KI          2.0f//3.0f    // Интегральный коэффициент
// до этого было значение 1.5f
#endif




// ==================== ФУНКЦИИ ====================

// Инициализация ШИМ (без включения выхода на пин)
// frequency_hz - частота ШИМ в Гц (например, 20000)
// max_duty - максимальное значение скважности (например, 1000)
void pwm_init();
uint16_t pwm_get_duty(void);
// Установка скважности (0 - выключено, max_duty - максимально)
void pwm_set_duty(uint16_t duty);

void pi_init();

uint16_t pi_compute(uint16_t setpoint, float measurement);

// Сброс интегральной составляющей ПИ-регулятора
void pi_reset_integral(void);

// Установка коэффициентов ПИ-регулятора "на лету"
void pi_set_gains(float new_kp, float newki);

#endif // PWM_H