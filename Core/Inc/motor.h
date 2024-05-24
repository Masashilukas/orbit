#pragma once

#include "stdbool.h"
#include  "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_spi.h"

typedef struct {
    GPIO_TypeDef * gpioGroup;
    uint16_t gpioPin;
} gpio_Pin;


typedef struct {
    TIM_HandleTypeDef* pwm;
    gpio_Pin hallPins[3];          // Hall channels for U, V, W pins
    gpio_Pin enablePins[3];
    gpio_Pin motorSleep;
    ADC_HandleTypeDef* adc;
    uint32_t adcData[3];
    SPI_HandleTypeDef* encoder;
    uint16_t encoderStartVal;
    uint8_t hallState;
    double speed;					// Speed calculated using hall sensors
    double angle;
    float iuDat;
    float ivDat;
    float iwDat;
    int adcSamples;
    int hallCount;
    float offset[3];
    float dutyCycle;
    float accumulatedCurrent;
    float hallspeed;
    float averageCurrent;
    bool driveStateChanged;
    int calibrationCount;
    int invalidCts;
    bool currentIsCalibrated[3];
    bool dir;
} motor_t;

void MOTOR_init(motor_t* m);
void MOTOR_updateShaftAngle(motor_t* m, int angle);
void MOTOR_task(motor_t* m);
void MOTOR_FOCtask(motor_t* m);
float MOTOR_getCurrent(motor_t* m);
void readHalls(motor_t* m);
void MOTOR_SVPWMtask(motor_t* m);
void _adcHandler(motor_t* m);
void _adcSelV(motor_t* m);
void _adcSelW(motor_t* m);
void _adcSelU(motor_t* m);
