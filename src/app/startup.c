/*
 * Copyright (c) 2023 Diego Asanza <f.asanza@gmail.com>
 * Created on Mon May 29 2023
 */

/* SPDX-License-Identifier: BSD 3-Clause  */

#include <stdint.h>
#include <hal_system.h>

#define __unused __attribute__((unused))
#define ALIAS(f) __attribute__((weak, alias(#f)))
#define REG(x) *((unsigned int *)x)

#define VTOR  0xE000ED08
#define CPACR 0xE000ED88
#define ICSR  0xE000ED04
#define CCR   0xE000ED14

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __data_start__;
extern unsigned int __data_end__;
extern unsigned int __etext;

void Default_Handler( void ) {
    /*
     * If we are here, chances are that we triggered an unhandled exception
     * handler. Read the active interrupt number bellow.
     */
    volatile __unused uint32_t vector = REG(ICSR) & 0xFFU;
    while(1);
}

void Reset_Handler( void );

extern int main( void );

extern void __StackTop(void);

void NMI_Handler( void )        ALIAS(Default_Handler);
void MemManage_Handler(void)    ALIAS(Default_Handler);
void BusFault_Handler(void)     ALIAS(Default_Handler);
void UsageFault_Handler( void ) ALIAS(Default_Handler);
void SecureFault_Handler(void)  ALIAS(Default_Handler);
void SVC_Handler( void )        ALIAS(Default_Handler);
void DebugMon_Handler( void )   ALIAS(Default_Handler);
void PendSV_Handler( void )     ALIAS(Default_Handler);
void SysTick_Handler( void )    ALIAS(Default_Handler);
void HardFault_Handler( void )  ALIAS(Default_Handler);


__attribute__((
    used, section(".isr_vector"))) void (*const g_interrupt_vector[])(void) = {
    &__StackTop,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void Reset_Handler( void )
{
    asm volatile("cpsid i");
    
    /* Remap the interrupt vector */
    REG(VTOR) = (unsigned int)g_interrupt_vector;

    /* initialize clock and other stuff. */
    hal_system_init();

    /* enable div by zero trap */
    REG(CCR) |= ( 1 << 4 );

    /* now we initialize data in ram from values saved in flash */
    volatile unsigned int *it = &__data_start__;
    volatile unsigned int *dr = &__etext;

    while (it < &__data_end__) {
        *it++ = *dr++;
    }

    /* clear the bss section */
    it = &__bss_start__;
    while (it < &__bss_end__) {
        *it++ = 0;
    }

    asm volatile("cpsie i");

    main();
    while(1);
}
