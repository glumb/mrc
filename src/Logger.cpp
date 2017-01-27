#include "Logger.h"


// 0> ONLY TEST 1> ERROR 2> WARNING 3> INFO
#define LOG_LEVEL 1
#define DEBUG 1

// #define DEBUG_NAMESPACE "handleSerialCommand"
// #define DEBUG_NAMESPACE "RobotController"
// #define DEBUG_NAMESPACE "main"

unsigned long Logger::usTime              = micros();
unsigned long Logger::usTimeSinceLastCall = micros();

Logger::Logger(String domain) : domain(domain) {}

void Logger::time(String eventName) {
  #ifdef DEBUG
    unsigned long tmpTime       = micros();
    unsigned long timeDiffStart = tmpTime - this->usTime;
    unsigned long timeDiffCall  = tmpTime - this->usTimeSinceLastCall;

    Serial.print(eventName);
    Serial.print(" cycle ");
    Serial.print(timeDiffStart);
    Serial.print(" diff ");
    Serial.print(timeDiffCall);
    Serial.println("");

    usTimeSinceLastCall = micros();

    // account for serial out time
    this->usTime             += micros() - tmpTime;
    this->usTimeSinceLastCall = micros();
  #endif // ifdef DEBUG
}

void Logger::resetTime() {
  #ifdef DEBUG
    this->usTime              = micros();
    this->usTimeSinceLastCall = micros();
  #endif // ifdef DEBUG
}

void Logger::error(String text, bool test) {
  #if LOG_LEVEL > 0
    this->log(ERROR, text, test);
  #endif
}

void Logger::warning(String text, bool test) {
  #if LOG_LEVEL > 1
    this->log(WARNING, text, test);
  #endif
}

void Logger::info(String text, bool test) {
  #if LOG_LEVEL > 2
    this->log(INFO, text, test);
  #endif
}

void Logger::info(unsigned char *text, bool test) {
  #if LOG_LEVEL > 2
    this->log(INFO, text, test);
  #endif
}

void Logger::log(LOGLEVEL lvl, String text, bool test) {
  #ifdef DEBUG


    # ifdef DEBUG_NAMESPACE

    if (!this->domain.equals(DEBUG_NAMESPACE)) return;

    # endif // ifdef NAMESPACE

    switch (lvl) {
    case ERROR:

        if ((LOG_LEVEL > 0) || test) {
            Serial.print(this->domain + " ERROR: ");
            Serial.println(text);
        }
        break;

    case WARNING:

        if ((LOG_LEVEL > 1) || test) {
            Serial.print(this->domain + " WARING: ");
            Serial.println(text);
        }
        break;

    case INFO:

        if ((LOG_LEVEL > 2) || test) {
            Serial.print(this->domain + " INFO: ");
            Serial.println(text);
        }
        break;
    }

    delay(20);

  #endif // ifdef (DEBUG)
}

void Logger::log(LOGLEVEL lvl, unsigned char *text, bool test) {
  #ifdef DEBUG

    # ifdef DEBUG_NAMESPACE

    if (!this->domain.equals(DEBUG_NAMESPACE)) return;

    # endif // ifdef NAMESPACE

    switch (lvl) {
    case ERROR:
    {
        if ((LOG_LEVEL > 0) || test) {
            Serial.print(this->domain + " ERROR: ");
            unsigned int i = 0;

            while (text[i] !=  0) {
                Serial.print((char)text[i]);
                i++;
            }
            Serial.println("");
        }
        break;
    }

    case WARNING:
    {
        if ((LOG_LEVEL > 1) || test) {
            Serial.print(this->domain + " WARING: ");
            unsigned int i = 0;

            while (text[i] !=  0) {
                Serial.print((char)text[i]);
                i++;
            }
            Serial.println("");
        }
        break;
    }

    case INFO:
    {
        if ((LOG_LEVEL > 2) || test) {
            Serial.print(this->domain + " INFO: ");
            unsigned int i = 0;

            while (text[i] !=  0) {
                Serial.print((char)text[i]);
                i++;
            }
            Serial.println("");
        }
        break;
    }
    }

    delay(20);

   #endif // ifdef (DEBUG)
}
