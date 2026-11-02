#ifndef CONFIG_H
#define CONFIG_H

#ifndef DEFAULT_VERSION 
#define DEFAULT_VERSION 1
#endif

#define DEBUG_ENABLE
//#define REGULATOR_ENABLE
//#define CONTROL_BY_POT_ENABLE
#ifndef F_CPU 
#define F_CPU    16000000UL      
#endif

#ifndef MASTER_ADDRESS
#define MASTER_ADDRESS 0x02
#endif

#ifndef SLAVE_ADDRESS
#define SLAVE_ADDRESS 0x01  
#endif

#ifndef BAUD_RATE
#define BAUD_RATE 9600    
#endif

#ifndef DEFAULT_STATUS
#define DEFAULT_STATUS 0x00
#endif

#ifndef DEFAULT_CAL_ADC_VOLTAGE_VALUE
#define DEFAULT_CAL_ADC_VOLTAGE_VALUE 5.0f/1023.0f
#endif

#ifndef DEFAULT_CAL_OUT_VOLTAGE_VALUE
#define DEFAULT_CAL_OUT_VOLTAGE_VALUE 0.431f
#endif

#ifndef DEFAULT_CAL_OUT_VOLTAGE_OFFSET_VALUE
#define DEFAULT_CAL_OUT_VOLTAGE_OFFSET_VALUE 3.371f
#endif

#define LED_BUILTIN_PIN PB7
#define LED_BUILTIN_PORT PORTB
#define LED_BUILTIN_DDR DDRB

#ifndef DEFAULT_SETPOINT
#define DEFAULT_SETPOINT        30    // Уставка по умолчанию (30В)
#endif

#define VOLTAGE_STEP_BUTTON     5     // Шаг изменения уставки при нажатии кнопки (В)
#define DUTY_STEP_BUTTON 25

#ifndef DEFAULT_MODE
#define DEFAULT_MODE            0      // 0 - тестовый, 1 - рабочий
#endif
#ifndef VOLTAGE_MIN
#define VOLTAGE_MIN             30     // Минимальное напряжение (В)
#endif
#ifndef VOLTAGE_MAX
#define VOLTAGE_MAX             250    // Максимальное напряжение (В)
#endif

//режимы работы
#define MODE_TEST              0   // Тестовый: ручное управление скважностью
#define MODE_WORK              1   // Рабочий: ПИ-регулятор по напряжению



#define PERIODIC_SEND_MS        100    // Период отправки данных мастеру (мс)
#define PROCESS_BUTTONS_MS         5   // Период обработки входящих данных и кнопок (мс)
#define MEASURE_VOLTAGE_MS         1  // Период измерения напряжения (мс)
#define REGULATOR_UPDATE_MS        5 // Период изменения шим (мс)

//ошибки

#define EEPROM_ERROR                    0x00
#define OVERVOLTAGE_ERROR               0x01
#define UNDERVOLTAGE_ERROR              0x02
#define MODE_ENTRY_TIMEOUT_ERROR        0x03
#define UART_ERROR                      0x04

#define PACKET_TYPE 0x06 //0=ACK or 1 = NACK
#define ACK_FLAG 0x00
#define NACK_FLAG 0x01
#define MODE_BIT 0x07


#define NACK_WRONG_COMMAND              0x00
#define NACK_WRONG_PARAM                0x01
#define NACK_WRONG_PARITY               0x02


#define EDIT_ADC_CALIBRATION_COMMAND 0x10
#define EDIT_VOUT_CALIBRATION_COMMAND 0x20
#define GET_DATA_COMMAND 0x30
#define MODE_TOGGLE_COMMAND 0x40
#define START_ADC_CALIBRATION_COMMAND 0x50
#define START_VOUT_CALIBRATION_COMMAND 0x60
#define EDIT_SETPOINT_COMMAND 0x70
#define RESET_COMMAND 0xA0


#endif // CONFIG_H