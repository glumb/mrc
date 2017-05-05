#ifndef LOGGER_H
#define LOGGER_H 1

#include "Arduino.h"


class Logger {
public:

    explicit Logger(String domain) : domain(domain) {}

    enum LOGLEVEL { ERROR, WARNING, INFO };

    static unsigned long usTime;
    static unsigned long usTimeSinceLastCall;

    void resetTime();

    void time(String eventName);

    void error(String text,
               bool   test = false) {
        this->log(ERROR,text);
    }

    void warning(String text,
                 bool   test = false) {
        this->log(WARNING,text);
    }

    void info(String text,
              bool   test = false) {
        this->log(INFO,text);
    }

    void info(const char *text,
              bool        test = false) {
        this->log(INFO,text);
    }

    void doLog(bool yesNo) {
        this->enabled = yesNo;
    }

private:

    void log(LOGLEVEL lvl,
             String   text,
             bool     test = false) {
        if (!this->enabled) {
            return;
        }
        std::cout << text << '\n';
    }

    void log(LOGLEVEL    lvl,
             const char *text,
             bool        test = false) {
        this->log(lvl,String(text));
    }

    bool enabled = true;
    String domain;
};

#endif // ifndef LOGGER_H
