#include "Logger.h"


// 0> ONLY TEST 1> ERROR 2> WARNING 3> INFO
#define LOG_LEVEL 2
#define DEBUG 1

// #define DEBUG_NAMESPACE "handleSerialCommand"
// #define DEBUG_NAMESPACE "RobotController"
// #define DEBUG_NAMESPACE "main"



Logger::Logger(String domain) : domain(domain) {}

void Logger::time(String eventName) {

}

void Logger::resetTime() {

}

void Logger::error(String text, bool test) {

}

void Logger::warning(String text, bool test) {

}

void Logger::info(String text, bool test) {

}

void Logger::info(unsigned char *text, bool test) {

}

void Logger::log(LOGLEVEL lvl, String text, bool test) {

}

void Logger::log(LOGLEVEL lvl, unsigned char *text, bool test) {

}
