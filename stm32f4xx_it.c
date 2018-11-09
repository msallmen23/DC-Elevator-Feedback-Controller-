/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"

static FlagStatus       em_flag = RESET;
static uint32_t         em_captureCount[2] = {0, 0};
static __IO uint32_t    em_capture = 0;

extern __IO uint32_t    em_inpFreq;
extern __IO uint32_t    m_pulse;

void TIM2_IRQHandler(void)
{
	/**
	 * Channel 3 - Encoder Module (INPUT)
	 */
	if(TIM_GetITStatus(TIM2, TIM_IT_CC3)) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

		if(!em_flag) {
			// Get the first rising edge count
			em_captureCount[0] = TIM_GetCapture3(TIM2);
			em_flag = SET;
		} else {
			// Get the second rising edge count
			em_captureCount[1] = TIM_GetCapture3(TIM2);

			if (em_captureCount[1] > em_captureCount[0]) {
				em_capture = (em_captureCount[1] - em_captureCount[0]);
			} else if (em_captureCount[1] < em_captureCount[0]) {
				em_capture =
					((0xFFFFFFFF - em_captureCount[0]) + em_captureCount[1]);
			} else {
				em_capture = 0;
			}

			// Compute the encoder module frequency
			em_inpFreq = (uint32_t) ((SystemCoreClock / 2) / em_capture) + 1;
			em_flag = RESET;
		}
	}
}

void TIM3_IRQHandler(void)
{
	/**
	 * Channel 3 - Motor Control (OUTPUT)
	 */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3)) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		// Set the PWM duty cycle adjusted by motorDutyCtrl()
		TIM_SetCompare3(TIM3, m_pulse);
	}
}

