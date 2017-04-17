#include <Arduino.h>
#include "../src/IOLogic.h"


Logger mainLogger("main");

void setup()
{
    Serial.begin(9600)

}

void loop()
{
  Serial.println("test");
  delay(1000);

}
