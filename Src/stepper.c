//////steppermotor control
#include "stm32f4xx_hal.h"
#include "stepper.h"

float cnt = 2.0;
int T = 105;
int32_t P = 125;///6400hz
int S = 200;
double rps = 0.0; ///revolute per second
double rpm = 0.0;
double frequency = 0.0;
bool dir;
double speed;
bool status = 1;

const bool CW = 1;
const bool CCW = 0;

double P_gain = 1.0;

void step_control(double err_value)
{
  rpm = P_gain*err_value;
  rps = (fabs(rpm)/60.0);
  frequency = 6400.0*rps;
  P = (int)(125.0/rps) ;
  
  if (rpm<0)
  {
    dir = CW;
  }
  else
  {
    dir = CCW;
  }
  
  if (dir == CW)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  }
  
  //////////60 rpm  ==> 6400HZ psc105 arr125
  /////300 rpm ==> psc105 arr25
  /////
  
}
void status_con()
{
  if(rpm == 0.0)
  {
    status = 1;
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
  }
  else
  {
    status = 0;
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
  }
}