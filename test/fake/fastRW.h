#ifndef FASTRW_H
#define FASTRW_H 1

#include <Arduino.h>



# define pinAsInput(P) pinMode(P, INPUT)
# define pinAsInputPullUp(P) pinMode(P, IMPUT_PULLUP)
# define pinAsOutput(P) pinMode(P, OUTPUT)
# define digitalLow(P) digitalWrite(P, LOW)
# define digitalHigh(P) digitalWrite(P, HIGH)
# define isHigh(P) digitalRead(P)
# define isLow(P) !digitalRead(P)
# define digitalState(P) digitalRead(P)




#endif /* ifndef FASTRW_H */
