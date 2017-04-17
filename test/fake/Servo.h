#ifndef SERVO_H
#define SERVO_H 1

class Servo {
public:

    Servo(){};
    void attach(int pin){};
    void write(int ms){};
    void writeMicroseconds(int ms){};

private:
};

#endif // ifndef SERVO_H
