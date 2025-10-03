/* Includes
 * ------------------------------------------------------------------*/
#include "at32f421_it.h"
#include "at32f421.h"

#include "ADC.h"
#include "main.h"
#include "targets.h"
#include "common.h"
#include "comparator.h"


// 26/9/2025 : 
//moved the handling of dshot and PWM inout calculation to exint 3 handle iso exint15
// exint15 irq is also used by sport telemetry on f421
// this way dshot can interrupt telemetry if needed



// Millisecond counter for system timing
volatile uint32_t system_millis = 0;


#include "singlewire_uart.h"


// Compiler messages to show which timers are being used
#ifdef USE_TIMER_15_CHANNEL_1
#pragma message("Using TIMER15 for input capture")
#endif

#ifdef USE_TIMER_3_CHANNEL_1
#pragma message("Using TIMER3 for input capture")
#endif

extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void signalEdgeRoutine();
extern void processDshot();

extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot;
int exti_int = 0;

void SysTick_Handler(void)
{
   system_millis++;
}

uint32_t getSystemMillis(void)
{
    return system_millis;
}
void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) { }

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) { }

/**
 * @brief  This function handles PendSV_Handler exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) { }

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */



void DMA1_Channel1_IRQHandler(void)
{
    if (dma_flag_get(DMA1_FDT1_FLAG) == SET) {
        DMA1->clr = DMA1_GL1_FLAG;
#ifdef USE_ADC
        ADC_DMA_Callback();
#endif
        if (dma_flag_get(DMA1_DTERR1_FLAG) == SET) {
            DMA1->clr = DMA1_GL1_FLAG;
        }

    }
}

void DMA1_Channel3_2_IRQHandler(void)
{
   
    // Handle DMA1_CHANNEL3 completion (Sport UART)
    if ((dma_flag_get(DMA1_FDT3_FLAG) == SET) | (dma_flag_get(DMA1_HDT3_FLAG) == SET))
    {

        DMA1->clr = DMA1_GL3_FLAG;
        DMA1_CHANNEL3->ctrl_bit.chen = FALSE;

    }
    // Handle DMA1_CHANNEL2 completion
    if (dma_flag_get(DMA1_FDT2_FLAG) == SET)
    {
        DMA1->clr = DMA1_GL2_FLAG;
        DMA1_CHANNEL2->ctrl_bit.chen = FALSE;
    }
    if (dma_flag_get(DMA1_DTERR2_FLAG) == SET)
    {
        DMA1->clr = DMA1_GL2_FLAG;
        DMA1_CHANNEL2->ctrl_bit.chen = FALSE;
    }
}


void DMA1_Channel5_4_IRQHandler(void)
{
    // lets just call the handler
    // it will return immediately when it is not initialized
    sw_uart_dma_complete_handler();


#ifdef USE_TIMER_15_CHANNEL_1
    if (dshot) {
        DMA1->clr = DMA1_GL5_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;
        return;
    }
    //    if (dma_flag_get(DMA1_HDT5_FLAG) == SET) {
    //        if (servoPwm) {
    //            IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
    //            DMA1->clr = DMA1_HDT5_FLAG;
    //        }
    //    }

    if (dma_flag_get(DMA1_FDT5_FLAG) == SET) {
        DMA1->clr = DMA1_GL5_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;
    }
    if (dma_flag_get(DMA1_DTERR5_FLAG) == SET) {
        DMA1->clr = DMA1_GL5_FLAG;
    }
#endif
#ifdef USE_TIMER_3_CHANNEL_1
    if (dshot) {
        DMA1->clr = DMA1_GL4_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_3;
        return;
    }
    if (dma_flag_get(DMA1_HDT4_FLAG) == SET) {
        if (servoPwm) {
            IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
            DMA1->clr = DMA1_HDT4_FLAG;
        }
    }
    if (dma_flag_get(DMA1_FDT4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_3;
    }
    if (dma_flag_get(DMA1_DTERR4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;
    }
#endif
}

/**
 * @brief This function handles ADC and COMP interrupts (COMP interrupts
 * through EXTI lines 21 and 22).
 */
void ADC1_CMP_IRQHandler(void)
{
  if((INTERVAL_TIMER->cval) > ((average_interval>>1))){
       EXINT->intsts = EXTI_LINE;
       interruptRoutine();
    }else{ 
      if (getCompOutputLevel() == rising){
        EXINT->intsts = EXTI_LINE;
    }
  }
}

/**
 * @brief This function handles TIM6 global and DAC underrun error interrupts.
 */
void TMR14_GLOBAL_IRQHandler(void)
{
    TMR14->ists = (uint16_t)~TMR_OVF_FLAG;
    tenKhzRoutine();
}

/**
 * @brief This function handles TIM14 global interrupt.
 */
void TMR16_GLOBAL_IRQHandler(void)
{
    TMR16->ists = 0x00;
    PeriodElapsedCallback();
}

void TMR15_GLOBAL_IRQHandler(void)
{
    TMR15->ists = (uint16_t)~TMR_OVF_FLAG;
    TMR15->ists = (uint16_t)~TMR_C1_FLAG;
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up
 * interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

void TMR3_GLOBAL_IRQHandler(void)
{
    if ((TMR3->ists & TMR_C1_FLAG) != (uint16_t)RESET) {
        TMR3->ists = (uint16_t)~TMR_C1_FLAG;
        // Handle input capture if TMR3 is used for RC input
    }
    if ((TMR3->ists & TMR_OVF_FLAG) != (uint16_t)RESET) {
        TMR3->ists = (uint16_t)~TMR_OVF_FLAG;
        // Handle timer overflow for TMR3
    }
}

/**
 * @brief  This function handles TMR6 global interrupt
 */
void TMR6_GLOBAL_IRQHandler(void)
{
    if ((TMR6->ists & TMR_OVF_FLAG) != (uint16_t)RESET)
    {
        TMR6->ists = (uint16_t)~TMR_OVF_FLAG;
    }
}

/**
 * @brief  This function handles TMR17 global interrupt
 */
void TMR17_GLOBAL_IRQHandler(void)
{
    if ((TMR17->ists & TMR_OVF_FLAG) != (uint16_t)RESET)
    {
        TMR17->ists = (uint16_t)~TMR_OVF_FLAG;
    }
}






// void DMA_Channel0_IRQHandler(void)         // ADC
//{
//	  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA global interrupt */
//	    /* (global interrupt flag: half transfer and transfer complete
// flags) */ 	    LL_DMA_ClearFlag_GI1(DMA1); 	    ADC_DMA_Callback();
//	    /* Call interruption treatment function */
//	 //   AdcDmaTransferComplete_Callback();
//	  }

//	  /* Check whether DMA transfer error caused the DMA interruption */
//	  if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA transfer error */
//	    LL_DMA_ClearFlag_TE1(DMA1);

//	    /* Call interruption treatment function */
//	  }
//}

void EXINT15_4_IRQHandler(void)
{

    // Handle Sport telemetry EXTI line 6 (PB6)
    if ((EXINT->intsts & EXINT_LINE_6) != (uint32_t)RESET)
    {
        // lets first handle the interrupt
        // and only then clear the flag
        // so we're sure that no other interrupt
        // is triggered during this process
        EXINT->intsts = EXINT_LINE_6;
        // the handle will return if no sport telemetry was active.
        sw_uart_exti_handler();
    }
}

  void EXINT3_2_IRQHandler(void)
{
  exti_int++;

    // Handle DShot EXTI line 3
    if ((EXINT->intsts & EXINT_LINE_3) != (uint32_t)RESET)
    {
        EXINT->intsts = EXINT_LINE_3;

       processDshot();
 
    }
  }

/******************************************************************************/
/*                 AT32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the */
/*  available peripheral interrupt handler's name please refer to the startup
 */
/*  file (startup_at32f413_xx.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */

/**
 * @}
 */
