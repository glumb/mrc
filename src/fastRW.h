#ifndef UNIT_TEST
# ifndef FASTRW_H
#  define FASTRW_H 1

#  include <Arduino.h>

#  ifdef CORE_TEENSY

#   define pinAsInput(P) pinMode(P, INPUT)
#   define pinAsInputPullUp(P) pinMode(P, IMPUT_PULLUP)
#   define pinAsOutput(P) pinMode(P, OUTPUT)
#   define digitalLow(P) digitalWriteFast(P, LOW)
#   define digitalHigh(P) digitalWriteFast(P, HIGH)
#   define isHigh(P) digitalRead(P)
#   define isLow(P) !digitalRead(P)
#   define digitalState(P) digitalRead(P)

#  else /* ifdef CORE_TEENSY */

#   define portOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &PORTD : (((P) > 7 && (P) < 14) ? &PORTB : &PORTC))
#   define ddrOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &DDRD : (((P) > 7 && (P) < 14) ? &DDRB : &DDRC))
#   define pinOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &PIND : (((P) > 7 && (P) < 14) ? &PINB : &PINC))
#   define pinIndex(P) ((uint8_t)(P > 13 ? P - 14 : P& 7))
#   define pinMask(P) ((uint8_t)(1 << pinIndex(P)))

#   define pinAsInput(P) *(ddrOfPin(P))       &= ~pinMask(P)
#   define pinAsInputPullUp(P) *(ddrOfPin(P)) &= ~pinMask(P); digitalHigh(P)
#   define pinAsOutput(P) *(ddrOfPin(P))      |= pinMask(P)
#   define digitalLow(P) *(portOfPin(P))      &= ~pinMask(P)
#   define digitalHigh(P) *(portOfPin(P))     |= pinMask(P)
#   define isHigh(P) ((*(pinOfPin(P)) & pinMask(P)) > 0)
#   define isLow(P) ((*(pinOfPin(P)) & pinMask(P)) == 0)
#   define digitalState(P) ((uint8_t)isHigh(P))

#  endif /* ifdef CORE_TEENSY */


# endif  /* ifndef FASTRW_H */

#else // ifndef UNIT_TEST

# include "Arduino.h"
# define pinAsInput(P) pinMode(P, INPUT)
# define pinAsInputPullUp(P) pinMode(P, IMPUT_PULLUP)
# define pinAsOutput(P) pinMode(P, OUTPUT)
# define digitalLow(P) digitalWrite(P, LOW)
# define digitalHigh(P) digitalWrite(P, HIGH)
# define isHigh(P) digitalRead(P)
# define isLow(P) !digitalRead(P)
# define digitalState(P) digitalRead(P)
#endif // ifndef UNIT_TEST
