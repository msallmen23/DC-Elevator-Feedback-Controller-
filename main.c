/**
 * Pin Description
 * ----------------
 * Switches
 * PB11 -> OUT          (START switch)
 * PB12 -> OUT          (STOP switch)
 * ----------------
 * Main Components
 * PB10  -> AF, TIM2 IC3 (Encoder Module)
 * PB0   -> AF, TIM3 OC3 (Motor)
 */

/* Includes */
#include "main.h"

__IO uint32_t	em_inpFreq = 0;
__IO uint32_t	m_pulse = 0;

void nvic_config(uint32_t TIMx_IRQn, uint32_t priorityLvl);
void pins_cfg(void);
void motor_cfg(void);
void motor_setSpeed(const uint32_t encModFreq);
void motor_setPulse(const uint8_t dutyCycle);
void encoderMod_cfg(void);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	// enable clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// setup switch pins
	pins_cfg();

	// setup motor
	motor_cfg();

	// setup encoder module
	encoderMod_cfg();

	while (1)
	{
		// wait until START switch goes LOW
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) != Bit_RESET);

		// turn on motor for 50% duty cycle
		motor_setPulse(50);

		// wait until IR (STOP) switch is reset, goes LOW
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) != Bit_RESET) {
			// Change the motor duty cycle according to the encoder module frequency
			motor_setSpeed(em_inpFreq);
		}

		// turn off motor
		motor_setPulse(0);
	}
}

void nvic_config(uint32_t TIMx_IRQn, uint32_t priorityLvl)
{
  NVIC_InitTypeDef NVIC_IS;

  NVIC_IS.NVIC_IRQChannel = TIMx_IRQn;
  NVIC_IS.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_IS.NVIC_IRQChannelSubPriority = priorityLvl;
  NVIC_IS.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_IS);
}

void pins_cfg(void)
{
	/* System variables */
	GPIO_InitTypeDef GPIO_IS;

	/**
	* Setup Pins
	* PB11 => IN
	*/
	GPIO_IS.GPIO_Pin = GPIO_Pin_11;
	GPIO_IS.GPIO_Mode = GPIO_Mode_IN;
	GPIO_IS.GPIO_OType = GPIO_OType_PP;
	GPIO_IS.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_IS.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOB, &GPIO_IS);
}

void motor_cfg(void)
{
	/* System variables */
	GPIO_InitTypeDef        GPIO_IS;
	TIM_OCInitTypeDef       TIM_OCIS;
	TIM_TimeBaseInitTypeDef TIM_TBIS;

	/**
	* Setup Pins
	* PB0 => AF
	*/
	GPIO_IS.GPIO_Pin = GPIO_Pin_0;
	GPIO_IS.GPIO_Mode = GPIO_Mode_AF;
	GPIO_IS.GPIO_OType = GPIO_OType_PP;
	GPIO_IS.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_IS.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOB, &GPIO_IS);

	/**
	* Connect pins to TIM3
	* MOTOR_PIN => Input (Channel 3)
	*/
	GPIO_PinAFConfig(GPIOB, 0, GPIO_AF_TIM3);

	/**
	* Setup NVIC for TIM3
	*/
	nvic_config(TIM3_IRQn, 0);

	/**
	* Setup TIM3 - Motor Control (OUTPUT)
	*/
	TIM_TBIS.TIM_Period = 2800;
	TIM_TBIS.TIM_Prescaler = 0;
	TIM_TBIS.TIM_ClockDivision = 0;
	TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TBIS);

	TIM_OCIS.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCIS.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCIS.TIM_Pulse = 0;
	TIM_OCIS.TIM_OutputState = ENABLE;

	TIM_OC3Init(TIM3, &TIM_OCIS);

	TIM_Cmd(TIM3, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}

/**
 * Controls the duty cycle of the motor depending on the Encoder Module input.
 */
void motor_setSpeed(const uint32_t encModFreq)
{
  // Set the duty cycle of the Motor
  switch (encModFreq) {
    case 4700:
      motor_setPulse(90);
      break;
    case 4900:
      motor_setPulse(80);
      break;
    case 5100:
      motor_setPulse(70);
      break;
    case 5300:
      motor_setPulse(60);
      break;
    case 5500:
      motor_setPulse(50);
      break;
    case 5700:
      motor_setPulse(40);
      break;
    case 5900:
      motor_setPulse(30);
      break;
    case 6100:
      motor_setPulse(20);
      break;
    case 6300:
      motor_setPulse(10);
      break;
    default:
      break;
  }
}

/**
 * Sets the pulse width of the motor.
 */
void motor_setPulse(const uint8_t dutyCycle)
{
  if (dutyCycle <= 100) {
    m_pulse = (2800 * dutyCycle) / 100;
  } else {
    m_pulse = (2800 * 50) / 100;
  }
}

void encoderMod_cfg(void)
{
  /* System variables */
  GPIO_InitTypeDef        GPIO_IS;
  TIM_ICInitTypeDef       TIM_ICIS;

  /**
   * Setup Pins
   * PB10 => AF
   */
  GPIO_IS.GPIO_Pin = GPIO_Pin_10;
  GPIO_IS.GPIO_Mode = GPIO_Mode_AF;
  GPIO_IS.GPIO_OType = GPIO_OType_PP;
  GPIO_IS.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_IS.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(GPIOB, &GPIO_IS);

  /**
   * Connect pins to TIM2
   * ENC_MOD_PIN => Input (Channel 3)
   */
  GPIO_PinAFConfig(GPIOB, 10, GPIO_AF_TIM2);

  /**
   * Setup NVIC for TIM2
   */
  nvic_config(TIM2_IRQn, 1);

  /**
   * Setup TIM2 - Encoder Module (INPUT)
   */
  TIM_ICIS.TIM_Channel = TIM_Channel_3;
  TIM_ICIS.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICIS.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICIS.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICIS.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICIS);

  TIM_Cmd(TIM2, ENABLE);

  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
}

