/*********************************************************************
   This is an example for our Monochrome OLEDs based on SSD1306 drivers

   Pick one up today in the adafruit shop!
   ------> http://www.adafruit.com/category/63_98

   This example is for a 128x32 size display using I2C to communicate
   3 pins are required to interface (2 I2C and one reset)

   Adafruit invests time and resources providing this open source code,
   please support Adafruit and open-source hardware by purchasing
   products from Adafruit!

   Written by Limor Fried/Ladyada  for Adafruit Industries.
   BSD license, check license.txt for more information
   All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);


class Display {
public:

    Display() {}

    void begin(){
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    }

    void show() {
        display.display();
    }

    void clear() {
        display.clearDisplay();
    }

    void displayText(unsigned int x, unsigned int y, String text) {
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(x, y);
        display.println(text);
    }

    void yellowNumberTopRight(int number) {
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(120, 0);
        display.drawLine(display.width() - 12, 0, display.width() - 12, 7, WHITE);
        display.println(String(number));
    }

    float map_float2(float x,
                     float in_min,
                     float in_max,
                     float out_min,
                     float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void displayRobot(Kinematic *kin, RobotController *RoboCon, unsigned int x, unsigned int y, float scale, unsigned int position = 0) {
        float jointPositions[7][3] = {0}; // 7 elements - Euler angles ar the last
        float angles[6];
        RoboCon->getCurrentLogicAngles(angles);
        kin->calculateCoordinates(angles[0],
                                  angles[1],
                                  angles[2],
                                  angles[3],
                                  angles[4],
                                  angles[5], jointPositions);

        switch (position) {
        case 0:                                                        // Front

            for (size_t i = 0; i < 5; i++) {
                display.drawLine(x + jointPositions[i][0] * scale * 2, // times two, sonce pixels are not square
                                 y + -jointPositions[i][1] * scale,
                                 x + jointPositions[i + 1][0] * scale * 2,
                                 y + -jointPositions[i + 1][1] * scale,
                                 WHITE);
            }
            break;

        case 1:                                                        // Top

            for (size_t i = 0; i < 5; i++) {
                display.drawLine(x + jointPositions[i][0] * scale * 2, // times two, sonce pixels are not square
                                 y + -jointPositions[i][2] * scale,
                                 x + jointPositions[i + 1][0] * scale * 2,
                                 y + -jointPositions[i + 1][2] * scale,
                                 WHITE);
            }
            break;

        case 2:                                                        // Side

            for (size_t i = 0; i < 5; i++) {
                display.drawLine(x + jointPositions[i][2] * scale * 2, // times two, sonce pixels are not square
                                 y + -jointPositions[i][1] * scale,
                                 x + jointPositions[i + 1][2] * scale * 2,
                                 y + -jointPositions[i + 1][1] * scale,
                                 WHITE);
            }
            break;
        }


    }

    void displayRobotGeometry(float        geometry[5][3],
                              unsigned int xPos = 30,
                              unsigned int yPos = 32,
                              unsigned int maxWidth = 64,
                              unsigned int maxHeight = 20) {
        // scale the model to 20 px height or 64px width
        float xMax  = 0;
        float xMin  = 0;
        float yMax  = 0;
        float yMin  = 0;
        float xTemp = 0;
        float yTemp = 0;

        // calculate max Dimensions
        for (size_t i = 0; i < 4; i++) {
            xTemp += geometry[i][0];
            yTemp += geometry[i][1];

            if (xTemp > xMax) {
                xMax = xTemp;
            } else if (xTemp < xMin) {
                xMin = xTemp;
            }

            if (yTemp > yMax) {
                yMax = yTemp;
            } else if (yTemp < yMin) {
                yMin = yTemp;
            }
        }

        int width  = xMax - xMin;
        int height = yMax - yMin;

        float robotDisplayScale;

        // wider aspect ratio
        if ((float)width / height > (float)maxWidth / maxHeight) {
            robotDisplayScale = (float)maxWidth  / width;
        } else {
            robotDisplayScale = (float)maxHeight / height;
        }

        for (size_t i = 0; i < 5; i++) {
            display.drawLine(xPos * 2, // times two since pixels are not square
                             yPos,
                             (xPos + geometry[i][0] * robotDisplayScale) * 2,
                             yPos - geometry[i][1] * robotDisplayScale, WHITE);
            xPos += geometry[i][0] * robotDisplayScale;
            yPos -= geometry[i][1] * robotDisplayScale;
        }
        display.display();
    }

    // values {min valuetarget valuecurrent max}
    void displayBars(unsigned int x, unsigned int y, unsigned int width, unsigned int height, MyServo *servos[]) {
        for (size_t i = 0; i < 6; i++) {
            // float range        = servos[i]->getCurrentAngle() - servos[i]->getMinRadAngle();
            int neutral = (int)map_float2(servos[i]->getHomeRadAngle(), servos[i]->getMinRadAngle(), servos[i]->getMaxRadAngle(), 0, width);

            int mappedValueTarget = (int)map_float2(servos[i]->getTargetRadAngle(), servos[i]->getMinRadAngle(),
                                                    servos[i]->getMaxRadAngle(), 0, width);
            int mappedValueCurrent = (int)map_float2(servos[i]->getCurrentAngle(), servos[i]->getMinRadAngle(),
                                                     servos[i]->getMaxRadAngle(), 0, width);

            unsigned int heightPerBar = height / 6;
            unsigned int offsetLineOne;
            unsigned int offsetLineTwo;

            switch (height) {
            case 6:
                offsetLineOne = 0;
                offsetLineTwo = 0;
                break;

            case 12:

                offsetLineOne = 0;
                offsetLineTwo = 1;
                break;

            case 18:
            case 24:
                offsetLineOne = 1;
                offsetLineTwo = 2;
                break;

            default:
                offsetLineOne = 1;
                offsetLineTwo = 2;
            }

            // x, y, width, height
            display.fillRect(x + neutral, y + i * heightPerBar, 1, heightPerBar, WHITE);

            if (neutral - mappedValueTarget > 0) {
                // target angle
                display.fillRect(x + mappedValueTarget, y + offsetLineOne + i * heightPerBar, -mappedValueTarget + neutral, 1, WHITE);
            } else {
                // target angle
                display.fillRect(x + neutral, y + offsetLineOne + i * heightPerBar, mappedValueTarget - neutral, 1, WHITE);
            }

            if (neutral - mappedValueCurrent > 0) {
                // current angle
                display.fillRect(x + mappedValueCurrent, y + offsetLineTwo + i * heightPerBar, -mappedValueCurrent + neutral, 1, WHITE);
            } else {
                // current angle
                display.fillRect(x + neutral, y + offsetLineTwo + i * heightPerBar, mappedValueCurrent - neutral, 1, WHITE);
            }
        }
    }

    void drawFrame() {
        display.drawLine(0, 7, display.width() - 1, 7, WHITE);
    }
};
