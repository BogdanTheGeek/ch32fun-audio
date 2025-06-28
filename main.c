//------------------------------------------------------------------------------
//       Filename: main.c
//------------------------------------------------------------------------------
//       Bogdan Ionescu (c) 2025
//------------------------------------------------------------------------------
//       Purpose : Application entry point
//------------------------------------------------------------------------------
//       Notes : Audio needs to be packed into a special format.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module includes
//------------------------------------------------------------------------------
#include "ch32fun.h"
#include "log.h"
#include "samples.h"
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

//------------------------------------------------------------------------------
// User defines
//------------------------------------------------------------------------------
#define SAMPLES     1024
#define SAMPLE_RATE 32000
// Make sure you use the right DMA channel number for your PWM output
#define DMA_CHANNEL_NUM 3

//------------------------------------------------------------------------------
// Module constant defines
//------------------------------------------------------------------------------
#define _CONCAT(x, y)     x##y
#define CONCAT(x, y)      _CONCAT(x, y)
#define _CONCAT3(x, y, z) x##y##z
#define CONCAT3(x, y, z)  _CONCAT3(x, y, z)
#ifndef array_size
#define array_size(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif
#define FREQ_TO_TICKS(freq) ((FUNCONF_SYSTEM_CORE_CLOCK / (freq)) - 1)

#define TAG "main"

static_assert(SAMPLES % 8 == 0, "SAMPLES must be a multiple of 8");

#define DMA_CHANNEL    CONCAT(DMA1_Channel, DMA_CHANNEL_NUM)
#define DMA_IRQn       CONCAT3(DMA1_Channel, DMA_CHANNEL_NUM, _IRQn)
#define DMA_ISRHandler CONCAT3(DMA1_Channel, DMA_CHANNEL_NUM, _IRQHandler)

//------------------------------------------------------------------------------
// External variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// External functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module type definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module static variables
//------------------------------------------------------------------------------
static volatile uint32_t s_systickCount = 0;
static uint32_t s_buffer[SAMPLES] = {0};
static volatile size_t s_bufferOffset = 0;
static uint8_t newByte = 0;
static uint32_t count = 0;
static uint32_t countLast = 0;

//------------------------------------------------------------------------------
// Module static function prototypes
//------------------------------------------------------------------------------
static void SysTick_Init(void);
static void PWM_Init(void);
static inline void unpack4(uint32_t *dst, const uint8_t *src);

//------------------------------------------------------------------------------
// Module externally exported functions
//------------------------------------------------------------------------------

/**
 * @brief  Application entry point
 * @param  None
 * @return None
 */
int main(void)
{
    SystemInit();

    const bool debuggerAttached = !WaitForDebuggerToAttach(1000);
    if (debuggerAttached)
    {
        LOG_Init(eLOG_LEVEL_DEBUG, (uint32_t *)&s_systickCount);
    }
    else
    {
        LOG_Init(eLOG_LEVEL_NONE, (uint32_t *)&s_systickCount);
    }

    SysTick_Init();
    LOGI(TAG, "System started");
    LOGI(TAG, "Sample rate: %dHz", SAMPLE_RATE);

loop_start:
    LOGI(TAG, "Samples: %d", (array_size(samples) / 5) * 4);

    uint32_t bytes_extracted = 0;

    for (size_t i = 0; i < SAMPLES; i += 4)
    {
        unpack4(&s_buffer[i], &samples[bytes_extracted]);
        bytes_extracted += 5;
    }
    s_bufferOffset = SAMPLES / 2;

#if ENABLE_PROGRESS_LOG
    uint32_t lastTick = s_systickCount;
#endif

    PWM_Init();
    while (1)
    {
        const size_t offset = s_bufferOffset;

        for (size_t i = 0; i < SAMPLES / 2; i += 4)
        {
            if (bytes_extracted < array_size(samples))
            {
                unpack4(&s_buffer[i + offset], &samples[bytes_extracted]);
                bytes_extracted += 5;
            }
            else
            {
                // If we run out of samples, just fill the rest with zeros
                s_buffer[i + offset] = 0;
                s_buffer[i + 1 + offset] = 0;
                s_buffer[i + 2 + offset] = 0;
                s_buffer[i + 3 + offset] = 0;
            }
        }

        while (offset == s_bufferOffset)
        {
            // Wait for the DMA to finish processing the current buffer
#if ENABLE_PROGRESS_LOG
            if ((s_systickCount - lastTick) > 100)
            {
                LOGI(TAG, "filled %d/%d", bytes_extracted, array_size(samples));
                lastTick = s_systickCount;
            }
#endif
        }

        if (bytes_extracted >= array_size(samples))
        {
            const size_t lastOffset = s_bufferOffset;
            while (lastOffset == s_bufferOffset)
                ; // Wait for the DMA to finish processing the last buffer
            break;
        }
    }
    LOGI(TAG, "Done");

    // Disable TIM1
    TIM1->CTLR1 &= ~TIM1_CTLR1_CEN;
    DMA_CHANNEL->CFGR &= ~DMA_CFGR1_EN;
    NVIC_DisableIRQ(DMA_IRQn);

    LOGW(TAG, "You can press 'r' to resume, 'q' to reset the board");
    while (1)
    {
        switch (getchar())
        {
            case 'r':
                goto loop_start;
            case 'q':
                NVIC_SystemReset();
            case -1:
                break;
            default:
                break;
        }
        Delay_Ms(100);
    }
}

/**
 * @brief  Debugger input handler
 * @param  numbytes - the number of bytes received
 * @param  data - the data (8 bytes)
 * @return None
 */
void handle_debug_input(int numbytes, uint8_t *data)
{
    newByte = data[0];
    count += numbytes;
}

/**
 * @brief  Get the next character from the debugger
 * @param  None
 * @return the next character or -1 if no character is available
 * @note   This function will block for up to 100ms waiting for a character
 */
int getchar(void)
{
    const uint32_t timeout = 100;
    const uint32_t end = s_systickCount + timeout;
    while (count == countLast && (s_systickCount < end))
    {
        poll_input();
        putchar(0);
    }

    if (count == countLast)
    {
        return -1;
    }

    countLast = count;
    return newByte;
}

//------------------------------------------------------------------------------
// Module static functions
//------------------------------------------------------------------------------

/**
 * @brief  Enable the SysTick module
 * @param  None
 * @return None
 */
static void SysTick_Init(void)
{
    // Disable default SysTick behavior
    SysTick->CTLR = 0;

    // Enable the SysTick IRQ
    NVIC_EnableIRQ(SysTicK_IRQn);

    // Trigger an interrupt in 1ms
    SysTick->CMP = SysTick->CNT + (FUNCONF_SYSTEM_CORE_CLOCK / 1000) - 1; // 1ms tick

    // Start at zero
    s_systickCount = 0;

    // Enable SysTick counter, IRQ, HCLK/1
    SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
}

/**
 * @brief  SysTick interrupt handler
 * @param  None
 * @return None
 * @note   __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
    // Trigger an interrupt in 1ms
    SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / 1000) - 1;

    // Clear IRQ
    SysTick->SR = 0;

    // Update counter
    s_systickCount++;
}

void DMA_ISRHandler(void) __attribute__((interrupt)) __attribute__((section(".srodata")));
void DMA_ISRHandler(void)
{
    volatile int intfr = DMA1->INTFR;
    do
    {
        DMA1->INTFCR = CONCAT(DMA1_IT_GL, DMA_CHANNEL_NUM);

        // Gets called at the end-of-a frame.
        if (intfr & CONCAT(DMA1_IT_TC, DMA_CHANNEL_NUM))
        {
            // Swap buffers
            s_bufferOffset = 0;
            // funDigitalWrite(PA1, 0);
        }
        // Gets called halfway through the frame
        if (intfr & CONCAT(DMA1_IT_HT, DMA_CHANNEL_NUM))
        {
            s_bufferOffset = SAMPLES / 2;
            // funDigitalWrite(PA1, 1);
        }

        intfr = DMA1->INTFR;
    } while (intfr);
}

static void PWM_Init(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA;
    // Enable DMA
    RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

    funPinMode(PA1, GPIO_CFGLR_OUT_10Mhz_AF_PP);
    funPinMode(PA2, GPIO_CFGLR_OUT_10Mhz_AF_PP);

    // NOTE: The system can only DMA out at ~2.2MSPS.  2MHz is stable.
    // The idea here is that this copies, byte-at-a-time from the memory
    // into the peripheral addres.
    DMA_CHANNEL->CNTR = SAMPLES * 2; // Number of samples to transfer
    DMA_CHANNEL->MADDR = (uint32_t)s_buffer;
    DMA_CHANNEL->PADDR = (uint32_t)&TIM1->CH2CVR; // This is T1CH2 Compare Register.
    DMA_CHANNEL->CFGR =
        DMA_CFGR1_DIR |     // MEM2PERIPHERAL
        DMA_CFGR1_PL |      // High priority.
        DMA_CFGR1_MSIZE_0 | // 16-bit memory
        DMA_CFGR1_PSIZE_1 | // 32-bit peripheral
        DMA_CFGR1_MINC |    // Increase memory.
        DMA_CFGR1_CIRC |    // Circular mode.
        DMA_CFGR1_HTIE |    // Half-trigger
        DMA_CFGR1_TCIE |    // Whole-trigger
        DMA_CFGR1_EN;       // Enable

    NVIC_EnableIRQ(DMA_IRQn);
    DMA_CHANNEL->CFGR |= DMA_CFGR1_EN;

    // Reset TIM1 to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // CTLR1: default is up, events generated, edge align
    // SMCFGR: default clk input is CK_INT

    // Prescaler
    TIM1->PSC = 0x0000; // No prescaler, TIM1 clock is 48 MHz

    // Auto Reload - sets period
    TIM1->ATRLR = FREQ_TO_TICKS(SAMPLE_RATE);

    // Reload immediately
    TIM1->SWEVGR |= TIM1_SWEVGR_TG | TIM1_SWEVGR_UG; // Update and trigger DMA

    // Enable CH2 complementatary output, normal polarity
    TIM1->CCER |= TIM1_CCER_CC2E | TIM1_CCER_CC2NE;

    // CH2 Mode is output, PWM1 (CC2S = 00, OC2M = 110)
    TIM1->CHCTLR1 |= TIM1_CHCTLR1_OC2M_2 | TIM1_CHCTLR1_OC2M_1;

    // Set the Capture Compare Register value to off
    TIM1->CH2CVR = 0; // TIM1->ATRLR / 2; // Set to 50% duty cycle initially

    // TRGO on update event
    TIM1->CTLR2 = TIM1_CTLR2_MMS_1;

    // Enable TIM1 outputs
    TIM1->BDTR |= TIM1_BDTR_MOE;
    // Set deadtime
    TIM1->BDTR = (TIM1->BDTR & ~TIM1_BDTR_DTG) | (0x4 & TIM1_BDTR_DTG);

    TIM1->DMAINTENR |= TIM1_DMAINTENR_UDE | TIM1_DMAINTENR_CC2DE; // Trigger DMA on update event

    // Enable TIM1
    TIM1->CTLR1 |= TIM1_CTLR1_CEN;
}

/**
 * @brief  Unpack 4 samples from the source buffer into the destination buffer
 * @param  dst - pointer to the destination buffer
 * @param  src - pointer to the source buffer
 * @return None
 * @note
 *  Sample packing:
 *  Sample: [0][1][2][3]
 *  Bytes:  [00000000][0011 1111][1111 2222][2222 2233][3333 3333]
 */
static inline void unpack4(uint32_t *dst, const uint8_t *src)
{
    dst[0] = (src[0] << 2) | (src[1] >> 6);
    dst[1] = ((src[1] & 0x3F) << 4) | (src[2] >> 4);
    dst[2] = ((src[2] & 0x0F) << 6) | (src[3] >> 2);
    dst[3] = ((src[3] & 0x03) << 8) | src[4];
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
