/*
 * Copyright (c) 2023 Diego Asanza <f.asanza@gmail.com>
 * Created on Mon May 29 2023
 */

/* SPDX-License-Identifier: BSD 3-Clause  */

/* sbrk implementation to be used with freertos. */

#include <FreeRTOS.h>
#include <task.h>
#include <errno.h>


extern char __HeapBase;
extern char __HeapLimit;

//! non-reentrant sbrk uses is actually reentrant by using current context
// ... because the current _reent structure is pointed to by global _impure_ptr
char * sbrk(int incr) { return _sbrk_r(_impure_ptr, incr); }
//! _sbrk is a synonym for sbrk.
char * _sbrk(int incr) { return sbrk(incr); }

void * _sbrk_r(struct _reent *pReent, int incr) {
    static unsigned long used_memory;
    static char *currentHeapEnd = &__HeapBase;
    vTaskSuspendAll(); // Note: safe to use before FreeRTOS scheduler started, but not within an ISR
    if (currentHeapEnd + incr > &__HeapLimit) {
        // Ooops, no more memory available...
        #if( configUSE_MALLOC_FAILED_HOOK == 1 )
        {
            extern void vApplicationMallocFailedHook( void );
            vApplicationMallocFailedHook();
        }
        #elif defined(configHARD_STOP_ON_MALLOC_FAILURE)
            // If you want to alert debugger or halt...
            while(1) { __asm("bkpt #0"); } // Stop in GUI as if at a breakpoint (if debugging, otherwise loop forever)
        #else
            // Default, if you prefer to believe your application will gracefully trap out-of-memory...
            pReent->_errno = ENOMEM; // newlib's thread-specific errno
            xTaskResumeAll();  // Note: safe to use before FreeRTOS scheduler started, but not within an ISR;
        #endif
        return (char *)-1; // the malloc-family routine that called sbrk will return 0
    }
    // 'incr' of memory is available: update accounting and return it.
    char *previousHeapEnd = currentHeapEnd;
    currentHeapEnd += incr;
    used_memory += incr;
    xTaskResumeAll();  // Note: safe to use before FreeRTOS scheduler started, but not within an ISR
    return (char *) previousHeapEnd;
}

void __malloc_lock(struct _reent *p)   { configASSERT( !xPortIsInsideInterrupt() ); // Make damn sure no mallocs inside ISRs!!
                                               vTaskSuspendAll(); }
void __malloc_unlock(struct _reent *p) { (void)xTaskResumeAll();  }

// newlib also requires implementing locks for the application's environment memory space,
// accessed by newlib's setenv() and getenv() functions.
// As these are trivial functions, momentarily suspend task switching (rather than semaphore).
// Not required (and trimmed by linker) in applications not using environment variables.
// ToDo: Move __env_lock/unlock to a separate newlib helper file.
void __env_lock()    {       vTaskSuspendAll(); }
void __env_unlock()  { (void)xTaskResumeAll();  }
