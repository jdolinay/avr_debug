#include "avr8-stub.h"
#include <avr/interrupt.h>

#ifndef AVR_DEBUGGER_H_
#define AVR_DEBUGGER_H_


#ifdef __cplusplus
extern "C" {
#endif

#if defined(PLATFORMIO) && defined(__PLATFORMIO_BUILD_DEBUG__)
    #ifndef __DEBUG__
        #define __DEBUG__
    #endif
#endif

#if defined(__DEBUG__)
    #define DBG_EXEC(x) x
#else
    #define DBG_EXEC(x)
#endif

#define dbg_breakpoint() { \
    DBG_EXEC(breakpoint()); \
    }

#define dbg_init()       { \
    DBG_EXEC(debug_init()); \
    }

#define dbg_message(x) { \
    DBG_EXEC(debug_message(x)); \
    }

#define dbg_start()      { \
    DBG_EXEC(debug_init()); \
    DBG_EXEC(sei()); \
    DBG_EXEC(breakpoint()); \
    }

#ifdef __cplusplus
}
#endif


#endif /* AVR_DEBUGGER_H_ */