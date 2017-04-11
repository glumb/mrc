#ifndef FASTRW_H
#define FASTRW_H 1

#include <Arduino.h>

#ifdef CORE_TEENSY

# define pinAsInput(P) pinMode(P, INPUT)
# define pinAsInputPullUp(P) pinMode(P, IMPUT_PULLUP)
# define pinAsOutput(P) pinMode(P, OUTPUT)
# define digitalLow(P) digitalWriteFast(P, LOW)
# define digitalHigh(P) digitalWriteFast(P, HIGH)
# define isHigh(P) digitalRead(P)
# define isLow(P) !digitalRead(P)
# define digitalState(P) digitalRead(P)

#else /* ifdef CORE_TEENSY */

# define portOfPin(P)
# define ddrOfPin(P)
# define pinOfPin(P) 
# define pinIndex(P) ((uint8_t)(P > 13 ? P - 14 : P& 7))
# define pinMask(P) ((uint8_t)(1 << pinIndex(P)))

# define pinAsInput(P) *(ddrOfPin(P))       &= ~pinMask(P)
# define pinAsInputPullUp(P) *(ddrOfPin(P)) &= ~pinMask(P); digitalHigh(P)
# define pinAsOutput(P) *(ddrOfPin(P))      |= pinMask(P)
# define digitalLow(P) *(portOfPin(P))      &= ~pinMask(P)
# define digitalHigh(P) *(portOfPin(P))     |= pinMask(P)
# define isHigh(P) ((*(pinOfPin(P)) & pinMask(P)) > 0)
# define isLow(P) ((*(pinOfPin(P)) & pinMask(P)) == 0)
# define digitalState(P) ((uint8_t)isHigh(P))

#endif /* ifdef CORE_TEENSY */


#endif /* ifndef FASTRW_H */
