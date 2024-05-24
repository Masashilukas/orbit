#include "motor.h"
#include "stdbool.h"
#include "math.h"
#include <stdint.h>
#define MP6543_SHUNT_RESISTOR   6800.0f
#define MP6543_IGAIN            9200.0f
#define ADC_VREF                3.3f
#define CAL_CYCLES              2000
float CURRENT_FILTER_CONST =    7500.0f;
void _drive_u(motor_t* m);
void _drive_v(motor_t* m);
void _drive_w(motor_t* m);
void _drive_uv(motor_t* m);
void _drive_vw(motor_t* m);
void _drive_wu(motor_t* m);
uint16_t readEncoder(motor_t* m){
	uint16_t buffer;
	HAL_SPI_Receive(m->encoder, &buffer, 1, 100); // one 12 bit byte 10 ms timeout
	return buffer;
}
void getAngle(motor_t* m){
	readHalls(m);
	//uint16_t encoderVal;
	//encoderVal = readEncoder(m);
	//double angle =((double)(encoderVal/ 4096)*360.0f);
	//double angle = (motor.hallspeed * 2 * 3.14159);
	//m->angle = angle;
}
void readHalls(motor_t* m){
	int hall1 = HAL_GPIO_ReadPin(m->hallPins[0].gpioGroup, m->hallPins[0].gpioPin);
	int hall2 = HAL_GPIO_ReadPin(m->hallPins[1].gpioGroup, m->hallPins[1].gpioPin);
	int hall3 = HAL_GPIO_ReadPin(m->hallPins[2].gpioGroup, m->hallPins[2].gpioPin);

	//altered hall inputs	
    switch((hall3<<2)|(hall2<<1)|(hall1))
    {
        case 0b100:
            m->angle = 0.0f;
            m->hallState = 0b100;
            break;
        case 0b110:
            m->angle = 60.0f;
            m->hallState = 0b110;
            break;
        case 0b010:
            m->angle = 120.0f;
            m->hallState = 0b010;
            break;
        case 0b011:
            m->angle = 180.0f;
            m->hallState = 0b011;
            break;
        case 0b001:
            m->angle = 240.0f;
            m->hallState = 0b001;
            break;
        case 0b101:
            m->angle = 300.0f;
            m->hallState = 0b101;
            break;
        default:
            m->invalidCts++;
            return;
    }
    //m->lastHallState = m->hallState;
  return;
}
void _adcHandler(motor_t* m)
{
    m->iuDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[0]/4096.0f)/MP6543_SHUNT_RESISTOR);
    m->ivDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[1]/4096.0f)/MP6543_SHUNT_RESISTOR);
    m->iwDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[2]/4096.0f)/MP6543_SHUNT_RESISTOR);
    //ADC data is taken in scanning mode resulting in a array of 3 adc values
    return;
}
/**
 * @brief Initialize the motor control structure
 *
 * @param m A pointer to a motor_t struct (not a vicproto MotorState!) that is already initialized with the correct references
 */
void MOTOR_init(motor_t* m)
{
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_3);
    //HAL_ADCEx_Calibration_Start(m->adc, ADC_SINGLE_ENDED);
    double temp = m->dutyCycle;
    m->dutyCycle = 1;
    m->pwm->Instance->CCR1 = round(255 * m->dutyCycle);
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    m->pwm->Instance->CCR2 = 0;
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    m->dutyCycle = temp;
    m->pwm->Instance->CCR1 = round(255 * m->dutyCycle);
    m->pwm->Instance->CCR2 = round(255 * m->dutyCycle);
    m->pwm->Instance->CCR3 = round(255 * m->dutyCycle);
    //while(m->angle != 0){
    //    MOTOR_task(m);
        //readHalls(m);
    //}
    //read encoder value for 0
    //m->encoderStartVal = readEncoder(m);
}
void MOTOR_updateShaftAngle(motor_t* m, int angle)
{
    m->angle = angle;
}
float MOTOR_getCurrent(motor_t* m)
{
    return m->averageCurrent;
}
void MOTOR_SVPWMtask(motor_t* m)
{
  	_adcSelU(m);
  	HAL_ADC_Start(m->adc);
  	HAL_ADC_PollForConversion(m->adc, 100);
  	m->iuDat = ((((3.3f * (double)HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
  	HAL_ADC_Stop(m->adc);
  	_adcSelV(m);
  	HAL_ADC_Start(m->adc);
  	HAL_ADC_PollForConversion(m->adc, 100);
  	m->ivDat = ((((3.3f * (double)HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
  	HAL_ADC_Stop(m->adc);
  	_adcSelW(m);
  	HAL_ADC_Start(m->adc);
  	HAL_ADC_PollForConversion(m->adc, 100);
  	m->iwDat = ((((3.3f *(double) HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
  	HAL_ADC_Stop(m->adc);
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
    //m->dir = m->motorState->pwm >= 0; // hall sensors are backwards

	if(m->dir){
    	m->pwm->Instance->CCR1 = round(255 * (0.5 * sin(m->angle  *  3.14159 / 180.0f) + 0.5)* m->dutyCycle); // pwmu = dutycycle
    	m->pwm->Instance->CCR2 = round(255 * (0.5 * sin((m->angle  *  3.14159 / 180.0f) - (2.0f * 3.14159 / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
    	m->pwm->Instance->CCR3 = round(255 * (0.5 * sin((m->angle  *  3.14159 / 180.0f) + (2.0f * 3.14159 / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
	}
	else{
		m->pwm->Instance->CCR1 = round(255 * (0.5 * sin(m->angle  *  3.14159 / 180.0f) + 0.5)* m->dutyCycle); // pwmu = dutycycle
    	m->pwm->Instance->CCR2 = round(255 * (0.5 * sin((m->angle  *  3.14159 / 180.0f) +  (2.0f * 3.14159 / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
    	m->pwm->Instance->CCR3 = round(255 * (0.5 * sin((m->angle  *  3.14159 / 180.0f) - (2.0f * 3.14159 / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
	}
}
void MOTOR_task(motor_t* m)
{
    //m->dir = m->motorState->pwm >= 0; // hall sensors are backwards
    if(m->angle < 60)
    {
    	if(m->dir) {
    		_drive_wu(m);
    		m->pwm->Instance->CCR1 = round(255 * m->dutyCycle); // pwmu = dutycycle
    		m->pwm->Instance->CCR3 = 0; // pwmw = 0
    	}
    	else {
      	   _drive_vw(m);
      	   m->pwm->Instance->CCR2 = round(255 * m->dutyCycle); // pwmw = dutycycle
      	   m->pwm->Instance->CCR3 = 0; // pwmv = 0
    	}
    }
    else if(m->angle < 120)
    {
        if(m->dir){
            _drive_vw(m);
			m->pwm->Instance->CCR2 = round(255 * m->dutyCycle); // pwmv = dutycycle
			m->pwm->Instance->CCR3 = 0; // pwmw = 0
        }
        else {
            _drive_uv(m);
            m->pwm->Instance->CCR2 = round(255 * m->dutyCycle); // pwmu = dutycycle
            m->pwm->Instance->CCR1 = 0; // pwmu= 0
        }
    }
    else if(m->angle < 180)
    {
        if(m->dir){
            _drive_uv(m);
			m->pwm->Instance->CCR2 = round(255 * m->dutyCycle); // pwmv = dutycycle
			m->pwm->Instance->CCR1 = 0; // pwmu = 0
        }
        else {
       	   _drive_wu(m);
      			m->pwm->Instance->CCR3 = round(255 * m->dutyCycle); // pwmu = dutycycle
      			m->pwm->Instance->CCR1 = 0; // pwmw = 0
        }
    }
    else if(m->angle < 240)
    {
    	if(m->dir){
    		_drive_wu(m);  // pwmu = dutycycle
    	    m->pwm->Instance->CCR3 = round(255 * m->dutyCycle); // pwmw = dutycycle
    	    m->pwm->Instance->CCR1 = 0; // pwmu = 0
    	}
    	else {
            _drive_vw(m);
 			m->pwm->Instance->CCR3 = round(255 * m->dutyCycle); // pwmv = dutycycle
 			m->pwm->Instance->CCR2 = 0; // pwmw = 0
    	}
    }
    else if(m->angle < 300)
    {
    	if(m->dir){
    	    _drive_vw(m);
    	    m->pwm->Instance->CCR3 = round(255 * m->dutyCycle); // pwmw = dutycycle
    	    m->pwm->Instance->CCR2 = 0; // pwmv = 0
    	}
    	else {
    		_drive_uv(m);
    		m->pwm->Instance->CCR1 = round(255 * m->dutyCycle); // pwmu = dutycycle
   			m->pwm->Instance->CCR2 = 0; // pwmw = 0
    	}
    }
    else if(m->angle < 360)
    {
    	if(m->dir){
            _drive_uv(m);
            m->pwm->Instance->CCR1 = round(255 * m->dutyCycle); // pwmu = dutycycle
			m->pwm->Instance->CCR2 = 0; // pwmv = 0
        }
    else {
     	   _drive_wu(m);  // pwmu = dutycycle
     	   m->pwm->Instance->CCR1 = round(255 * m->dutyCycle); // pwmw = dutycycle
     	   m->pwm->Instance->CCR3 = 0; // pwmu = 0
       }
    }
}
void _adcSelU(motor_t* m)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(m->adc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void _adcSelV(motor_t* m)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(m->adc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void _adcSelW(motor_t* m)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel =  ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(m->adc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void MOTOR_FOCtask(motor_t* m)
{    // enable all channels
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
    //closed loop until angle = 0;
	//while(m->angle != 0){
	//	MOTOR_task(m);
	//}
    // enable encoder and get accurate angle values
	_adcSelU(m);
	HAL_ADC_Start(m->adc);
	HAL_ADC_PollForConversion(m->adc, 100);
	m->iuDat = ((((double)(HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
	HAL_ADC_Stop(m->adc);
	_adcSelV(m);
	HAL_ADC_Start(m->adc);
	HAL_ADC_PollForConversion(m->adc, 100);
	m->ivDat = ((((double)(HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
	HAL_ADC_Stop(m->adc);
	_adcSelW(m);
	HAL_ADC_Start(m->adc);
	HAL_ADC_PollForConversion(m->adc, 100);
	m->iwDat = ((((double)(HAL_ADC_GetValue(m->adc))/ 4096.0f) * 9200.0f) / 6800.0f);
	HAL_ADC_Stop(m->adc);
	//m->angle += 0;
    double a = m->iuDat - (0.5f * m->ivDat) - (0.5f * m->iwDat);
    double b = (double)((0.866 * m->ivDat) - (0.866 * m->iwDat));
    // convert to direct and quadriture currents
    double id =  a * cos(((double)m->angle * 3.1415926 / 180.0f)) + b * sin(((double)m->angle * 3.1415926 / 180.0f));
    double iq = -a * sin(((double)m->angle * 3.1415926 / 180.0f)) + b * cos(((double)m->angle * 3.1415926 / 180.0f));
    //6 Convert to 2 phase voltages using inverse park transform (DOUBLE CHECK THIS)
    double newa = (cos(((double)m->angle * 3.1415926 / 180.0f)) * id) - (sin(((double)m->angle * 3.1415926 / 180.0f)) * iq);
    double newb = (sin(((double)m->angle * 3.1415926 / 180.0f)) * id) + (cos(((double)m->angle * 3.1415926 / 180.0f)) * iq);
    //7 Convert to 3 phase voltages using inverse clarke transform
    double U_dutyCycle = (double)(newa * 0.512f);
    double V_dutyCycle = (double)((-0.5f * newa) + ((sqrt(3.0f)/2.0f) * newb));
    double W_dutyCycle = (double)((-0.5f * newa) - ((sqrt(3.0f)/2.0f) * newb));
	if(U_dutyCycle < 0){
		U_dutyCycle = -U_dutyCycle;
	}
	else if(U_dutyCycle > 1){
		U_dutyCycle = 1;
	}
	if(V_dutyCycle < 0){
		V_dutyCycle = -V_dutyCycle;
	}
	else if(V_dutyCycle > 1){
		V_dutyCycle = 1;
	}
	if(W_dutyCycle < 0){
		W_dutyCycle = -W_dutyCycle;
	}
	else if(W_dutyCycle > 1){
		W_dutyCycle = 1;
	}

	m->pwm->Instance->CCR1 = round(255 * U_dutyCycle);
    m->pwm->Instance->CCR2 = round(255 * V_dutyCycle);
    m->pwm->Instance->CCR3 = round(255 * W_dutyCycle);
}
void _drive_u(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 0);
}
void _drive_v(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 0);
}
void _drive_w(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
}
void _drive_uv(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 0);
}
void _drive_vw(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
}
void _drive_wu(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
}