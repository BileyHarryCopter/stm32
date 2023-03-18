#include <stdint.h>
#include <stdbool.h>

//---------------
// RCC Registers
//---------------

#define REG_RCC_CR     (volatile uint32_t *)(uintptr_t)0x40021000U     // Clock Control Register
#define REG_RCC_CFGR   (volatile uint32_t *)(uintptr_t)0x40021004U     // PLL Configuration Register
#define REG_RCC_AHBENR (volatile uint32_t *)(uintptr_t)0x40021014U     // AHB1 Peripheral Clock Enable Register
#define REG_RCC_CFGR2  (volatile uint32_t *)(uintptr_t)0x4002102CU     // Clock configuration register 2

//----------------
// GPIO Registers
//----------------

#define GPIOC_MODER (volatile uint32_t *)(uintptr_t)0x48000800U // GPIO port mode register: we need general purpose output mode
#define GPIOC_TYPER (volatile uint32_t *)(uintptr_t)0x48000804U // GPIO port output type register: we need of push-pull
#define GPIOC_ODR   (volatile uint32_t *)(uintptr_t)0x48000814U // GPIO port output data register

//--------------
// REG Modifying
//--------------

#define REG_ADD_VALUE_BY_REG_IN_BIT(REG, VALUE, BIT) (*REG |= (VALUE << BIT))
#define REG_SET_ONE_IN_BIT(REG, BIT)                 (REG_ADD_VALUE_BY_REG_IN_BIT (REG, 1U, BIT))
#define REG_ADD_OR_EQ(REG, VALUE)                    (REG_ADD_VALUE_BY_REG_IN_BIT (REG, VALUE, 0U))
#define REG_SET_ZERO_IN_BIT(REG, BIT)                (*REG &= ~(1U << BIT))

//-----
// LEDS
//-----

#define LED_BLUE 8U
#define LED_GREEN 9U 

//------
// Main
//------

#define CPU_FREQENCY 48000000U // CPU frequency: 48 MHz
#define ONE_MILLISECOND CPU_FREQENCY / 1000U

void board_clocking_init()
{
    // (1) Clock HSE and wait for oscillations to setup.
    *REG_RCC_CR = 0x00010000U;
    while ((*REG_RCC_CR & 0x00020000U) != 0x00020000U) {;}

    // (2) Configure PLL:
    // PREDIV output: HSE/2 = 4 MHz
    REG_ADD_OR_EQ(REG_RCC_CFGR2, 1U);

    // (3) Select PREDIV output as PLL input (4 MHz):
    REG_ADD_OR_EQ(REG_RCC_CFGR, 0x00010000U);

    // (4) Set PLLMUL to 12:
    // SYSCLK frequency = 48 MHz
    REG_ADD_VALUE_BY_REG_IN_BIT(REG_RCC_CFGR, (12U - 2U), 18U);

    // (5) Enable PLL:
    REG_ADD_OR_EQ(REG_RCC_CR, 0x01000000U);
    while ((*REG_RCC_CR & 0x02000000U) != 0x02000000U) {;}

    // (6) Configure AHB frequency to 48 MHz:
    REG_ADD_VALUE_BY_REG_IN_BIT(REG_RCC_CFGR, 0U, 4U);

    // (7) Select PLL as SYSCLK source:
    REG_ADD_OR_EQ(REG_RCC_CFGR, 0x10U);
    while ((*REG_RCC_CFGR & 0xCU) != 0x8U) {;}

    // (8) Set APB frequency to 24 MHz
    REG_ADD_VALUE_BY_REG_IN_BIT(REG_RCC_CFGR, 1U, 8U);
}

void board_gpio_init()
{
    // (1) Enable GPIOC clocking:
    REG_ADD_OR_EQ(REG_RCC_AHBENR, 0x80000U);

    // (2) Configure PC8 mode:
    REG_SET_ONE_IN_BIT(GPIOC_MODER, 2 * 8U);

    // (3) Configure PC8 type:
    REG_SET_ZERO_IN_BIT(GPIOC_TYPER, 8U);

    // (4) Configure PC9 mode:
    REG_SET_ONE_IN_BIT(GPIOC_MODER, 2 * 9U);

    // (5) Configure PC9 type:
    REG_SET_ZERO_IN_BIT(GPIOC_TYPER, 9U);
}

void totally_accurate_quantum_femtosecond_precise_super_delay_3000_1000ms()
{
    for (uint32_t i = 0; i < 1000U * ONE_MILLISECOND; ++i)
    {
        // Insert NOP for power consumption:
        __asm__ volatile("nop");
    }
}

int main()
{
#ifndef INSIDE_QEMU
    board_clocking_init();
#endif

    board_gpio_init();

    for (; true;)
    {
        REG_SET_ONE_IN_BIT(GPIOC_ODR, LED_GREEN);

        totally_accurate_quantum_femtosecond_precise_super_delay_3000_1000ms();

        REG_SET_ZERO_IN_BIT(GPIOC_ODR, LED_GREEN);

        totally_accurate_quantum_femtosecond_precise_super_delay_3000_1000ms();

        REG_SET_ONE_IN_BIT(GPIOC_ODR, LED_BLUE);

        totally_accurate_quantum_femtosecond_precise_super_delay_3000_1000ms();

        REG_SET_ZERO_IN_BIT(GPIOC_ODR, LED_BLUE);

        totally_accurate_quantum_femtosecond_precise_super_delay_3000_1000ms();
    }
}
