/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 7;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 7;

// Bit-band alias region addresses for AT32F421
#define PERIPH_BB_BASE        0x42000000UL

// Calculate bit-band alias address using AT32F421 datasheet formula
// For peripheral bit-band: AliasAddr = 0x4200_0000 + (A-0x4000_0000)*32 + n*4
#define BITBAND_PERIPH(addr, bit) \
    ((PERIPH_BB_BASE + (((uint32_t)(addr) - PERIPH_BASE) << 5) + ((bit) << 2)))


// EXTI register addresses
#define EXTI_BASE             0x40010400UL
#define EXTI_INTEN_ADDR       (EXTI_BASE + 0x00)  // INTEN register offset
#define EXTI_EVTEN_ADDR       (EXTI_BASE + 0x04)  // EVTEN register offset
#define EXTI_POLCFG1_ADDR     (EXTI_BASE + 0x08)  // POLCFG1 register offset
#define EXTI_POLCFG2_ADDR     (EXTI_BASE + 0x0C)  // POLCFG2 register offset
#define EXTI_INTSTS_ADDR      (EXTI_BASE + 0x14)  // INTSTS register offset

void atomic_exti_bit_modify(uint32_t line, uint8_t state) {
    // Calculate bit-band address for specific EXTI line
    volatile uint32_t* bitband_addr = (volatile uint32_t*)BITBAND_PERIPH(EXTI_INTEN_ADDR, line);
    *bitband_addr = state;  // Atomic bit set/clear - cannot be interrupted

}
void atomic_event_bit_modify(uint32_t line, uint8_t state) {
    // Calculate bit-band address for specific EXTI line
    volatile uint32_t* bitband_addr = (volatile uint32_t*)BITBAND_PERIPH(EXTI_EVTEN_ADDR, line);
    *bitband_addr = state;  // Atomic bit set/clear - cannot be interrupted

}

void atomic_polcfg1_bit_modify(uint32_t line, uint8_t state) {
    // Calculate bit-band address for specific EXTI line
    volatile uint32_t* bitband_addr = (volatile uint32_t*)BITBAND_PERIPH(EXTI_POLCFG1_ADDR, line);
    *bitband_addr = state;  // Atomic bit set/clear - cannot be interrupted

}

void atomic_polcfg2_bit_modify(uint32_t line, uint8_t state) {
    // Calculate bit-band address for specific EXTI line
    volatile uint32_t* bitband_addr = (volatile uint32_t*)BITBAND_PERIPH(EXTI_POLCFG2_ADDR, line);
    *bitband_addr = state;  // Atomic bit set/clear - cannot be interrupted

}


void changeToOutput()
{
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
    tmr_reset(IC_TIMER_REGISTER);
    IC_TIMER_REGISTER->cm1 = 0x60; // oc mode pwm
    IC_TIMER_REGISTER->cctrl = 0x3; //
    IC_TIMER_REGISTER->div = output_timer_prescaler;
    IC_TIMER_REGISTER->pr = 76; // 76 to start

    out_put = 1;
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
}

void changeToInput()
{
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;
    tmr_reset(IC_TIMER_REGISTER);
    IC_TIMER_REGISTER->cm1 = 0x41;
    IC_TIMER_REGISTER->cctrl = 0xB;
    IC_TIMER_REGISTER->div = ic_timer_prescaler;
    IC_TIMER_REGISTER->pr = 0xFFFF;
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
    out_put = 0;
}
void receiveDshotDma()
{
    changeToInput();
    IC_TIMER_REGISTER->cval = 0;
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    INPUT_DMA_CHANNEL->dtcnt = buffersize;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    INPUT_DMA_CHANNEL->ctrl = 0x0000098b;
}

void sendDshotDma()
{
    changeToOutput();
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&gcr;
    INPUT_DMA_CHANNEL->dtcnt = 23 + buffer_padding;
    INPUT_DMA_CHANNEL->ctrl |= DMA_FDT_INT;
    INPUT_DMA_CHANNEL->ctrl |= DMA_DTERR_INT;
    INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    IC_TIMER_REGISTER->brk_bit.oen = TRUE;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
}

uint8_t getInputPinState()
{
    uint8_t state = INPUT_PIN_PORT->idt & INPUT_PIN;
    return state;
}

void setInputPolarityRising()
{
    IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
}

void setInputPullDown()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_DOWN, INPUT_PIN);
}

void setInputPullUp()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_UP, INPUT_PIN);
}

void enableHalfTransferInt() { INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT; }
void setInputPullNone()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
}
