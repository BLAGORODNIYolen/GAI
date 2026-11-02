// ==================== pwm.c ====================
#include "pwm.h"
#include "config.h"
#include <avr/io.h>

// ==================== СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ ====================

// ШИМ
static uint16_t current_duty = 0;           // Текущая скважность (0-max_duty)

// ПИ-регулятор
static float kp;                     // Пропорциональный коэффициент
static float ki;                     // Интегральный коэффициент
static float integral;                // Интегральная составляющая
static float integral_limit;                 // Предел интегральной составляющей
static float output;                // Выход регулятора (до масштабирования в скважность)
static uint16_t pi_min_output = 0;           // Минимальный выход
static uint16_t pi_max_output = PWM_MAX_AVAILABLE_DUTY;// Максимальный выход

static float error_abs(float value){
    return (value >= 0.0f) ? value : -value;
}

// ==================== ИНИЦИАЛИЗАЦИЯ ШИМ ====================

// Обновление значения скважности в регистр таймера
static void update_duty(void) {

    if (current_duty > PWM_MAX_AVAILABLE_DUTY) current_duty = PWM_MAX_AVAILABLE_DUTY;
    // Масштабируем значение скважности (0-max_duty) в значение для OCR1A (0-PWM_TOP)
    uint16_t ocr_value = (uint32_t)current_duty * PWM_TOP / PWM_MAX_DUTY;
    if (ocr_value > PWM_TOP) ocr_value = PWM_TOP;
    OCR1A = ocr_value;
}


void pwm_init() {//5kHz, 10 бит

    DDRB |= (1 << PWM_PIN);  // PB5 как выход
    // Изначально выход отключён (пин в 0)
    PORTB &= ~(1 << PWM_PIN);


    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1A |= (1 << WGM11); 
    TCCR1A |= (1 << COM1A1);                
    TCCR1B |= (1 << WGM12) | (1 << WGM13);  
    TCCR1B |= (1 << CS11);
    // Устанавливаем TOP значение
    ICR1 = PWM_TOP;
    TCNT1 = 0; // Сброс счётчика
    // Начальное значение скважности
    current_duty = 0;
    update_duty();
}

void pwm_set_duty(uint16_t duty) {
    current_duty = duty;
    update_duty();
}

uint16_t pwm_get_duty(void) {
    return current_duty;
}


// ==================== ИНИЦИАЛИЗАЦИЯ ПИ-РЕГУЛЯТОРА ====================

void pi_init() {
    ki = PI_KI;
    kp = PI_KP;
    // Это предотвращает глубокое насыщение интегратора (anti-windup)
    integral_limit = (float)pi_max_output * 0.8f;
    // Сбрасываем интегральную составляющую
    integral = 0.0f;

}

// ==================== ПИ-РЕГУЛЯТОР ====================
//возвращает скважность (0 - max_duty) для заданной уставки и измерения
uint16_t pi_compute(uint16_t setpoint, float measurement) {
    // Вычисляем ошибку
    float error = (float)setpoint - measurement;
    
    // Пропорциональная составляющая
    float proportional = kp * error;

    if (error_abs(error) > 0.7f) {
        integral += ki * error*REGULATOR_UPDATE_MS/1000.0f; // Учитываем время между обновлениями
    }
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
        
    // Интегральная составляющая (с защитой от насыщения - anti-windup)
    // Добавляем интеграл только если выход не насыщен
    output = proportional + integral;
    
    // Ограничиваем выход
    if (output > pi_max_output) output = pi_max_output;
    else if (output < pi_min_output) output = pi_min_output;


    return (uint16_t)output;
}

void pi_reset_integral(void) {
    integral = 0.0f;
}

void pi_set_gains(float new_kp, float new_ki) {
    kp = new_kp;
    ki = new_ki;
    
    // Пересчитываем предел интегральной составляющей
    integral_limit = (float)pi_max_output * 0.2f;
    
    // Сбрасываем интеграл для предотвращения выбросов
    pi_reset_integral();
}