#ifndef QCAR_TASKING_INTELLISENSE_H
#define QCAR_TASKING_INTELLISENSE_H

/* Parsing adapters only: never include this file in the ADS firmware build. */
#if defined(__INTELLISENSE__) || defined(QCAR_VSCODE_ANALYSIS)
#define __far
#define __near
#define __a0
#define __a1
#define __a8
#define __a9
#define __sat
#define __fract int
#define __sfract short
#define __laccum long long
#define __sfrbit16 short
#define __sfrbit32 int
#define __interrupt(priority)
#define __interrupt_fast(priority)
#define __vector_table(table)
#define __trap(number)
#define __trap_fast(number)
#define __align(n) aligned(n)
/* The editor cannot validate TriCore register constraints on a host compiler. */
#define __asm(...)

/* TASKING supplies these functions as compiler builtins, without headers. */
#ifdef __cplusplus
extern "C" {
#endif
void __debug(void);
void __disable(void);
void __enable(void);
void __dsync(void);
void __isync(void);
void __nop(void);
unsigned int __mfcr(unsigned int reg);
void __mtcr(unsigned int reg, unsigned int value);
int __clz(unsigned int value);
int __min(int a, int b);
int __max(int a, int b);
unsigned int __minu(unsigned int a, unsigned int b);
unsigned int __maxu(unsigned int a, unsigned int b);
unsigned int __extru(unsigned int value, unsigned int pos, unsigned int width);
unsigned int __insert(unsigned int a, unsigned int b, unsigned int pos, unsigned int width);
unsigned int __getbit(const volatile void *address, unsigned int offset);
void __imaskldmst(volatile void *address, unsigned int value, unsigned int pos, unsigned int width);
unsigned int __cmpswapw(volatile void *address, unsigned int value, unsigned int condition);
unsigned int __swap(volatile void *address, unsigned int value);
short __round16(int value);
#ifdef __cplusplus
}
#endif
#endif

#endif
