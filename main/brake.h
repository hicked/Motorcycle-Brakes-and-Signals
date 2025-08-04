#pragma once
#include <FastLED.h> // 3.9.9
#include "signals.h"
#include "gyro.h"
#include "button.h"

#define BRAKE_PIN                   11                  // Pin for the brake lights

#define BACKLIGHT_COLOUR            CRGB(8, 0, 0)       // Dim red for the background
#define ACTIVE_BRAKE_COLOUR         CRGB(this->active_brightness, 0, 0) // Bright red for the active brake
#define ACTIVE_ACC_COLOUR           CRGB(0, this->active_brightness, 0) // Green for acceleration

#define FLASHLIGHT_COLOUR           CRGB(255, 255, 255) // white
#define FLASH_COLOUR                CRGB(255, 0, 0)     // Colour when the entire brake is flashing
#define CENTER_FLASH_COLOUR         CRGB(255, 0, 0)     // Colour when the center is flashing

#define GLOBAL_BRIGHTNESS           255                 // 0-255, this might be broken since it gets overriden
#define MIN_BRAKE_BRIGHTNESS        200                 // 0-255 minimum brightness of the brake when braking
#define MAX_BRAKE_BRIGHTNESS        255                 // 0-255 maximum brightness of the brake when braking

#define CENTER_FLASH_SPEED          50                  // flashrate of the center part lower is faster
#define CENTER_FLASH_WIDTH          4                   // Width of the flashing center part of the brake. should be even

#define FLASH_SPEED                 25                  // delay between flashes for emergency braking and initialization of brakes
#define INI_BRAKING_FLASH_LENGTH    3                   // Number of brake flahses when initializing braking

#define TIME_BETWEEN_INI_BRAKE      3000                // initialization of braking detection can only happen every x milliseconds

#define SHOW_ACC                    false               // show acceleration (green)
#define SHOW_CHROMA                 false               // show chroma when accelerating fast (rainbow)

class Signals;

class Brake {
private:
    CRGB            *LEDStrip;                          // Pointer to the LED strip
    Signals         *signals;
    Gyro            *gyro;
    Button          *button;

    bool            flashON = false;                    // status while flashing
    bool            centerFlashON = false;              // status while center flashing

    unsigned long   lastFlashTime = 0;                  // time of the last flash
    unsigned long   lastCenterFlashTime = 0;            // time of the last center flash

    unsigned long   timeSinceLastIniBraking = 0;        // time since last initialized braking

    unsigned long   lastMarioTime = 0;                  // time of the last rainbow
    unsigned long   lastPatternTime = 0;
    unsigned long   lastMarqueeTime = 0;

    int             numLEDs;                            // number of LEDs, from header in main.ino
    int             middleIndex;                        // middle index of the LED strip (+1 since even)  xxxoxx



    unsigned long   cac = 0;


    
    void dynamicBrakeMode();                            // show brake lights based on how hard you brake
    void staticBrakeMode();

    void setSolid(CRGB colour);                         // set the entire LED strip to a single color
    void flashCenterLEDs();                             // flash the center LEDs

    void shiftPatternMode(CRGB colours[], 
                          int numColours,
                          unsigned long speed, 
                          int size, 
                          bool invertDirection=false);  // shift the pattern of the brake lights

    void marqueeEffect(CRGB* colours, 
                       int numColours, 
                       int speed, 
                       float blendFactor, 
                       bool fromCenter=true, 
                       bool invertDirection=false);     // show marquee effect on the brake lights

    /* 
    void marqueeEffect(CRGB* colours, 
                      int numColours, 
                      int speed, 
                      int sizeOfBlend, 
                      int sustain, 
                      bool fromCenter=true, 
                      bool invertDirection=false);
    */

    void christmasMode();                               // show christmas lights on the brake lights
    void halloweenMode();                               // show halloween lights on the brake lights
    void flashlightMode();                              // show flashlight on the brake lights

    CRGB blendColours(CRGB colour1, 
                      CRGB colour2, 
                      float blendFactor);               // blend two colours together

public:
    bool initializedBraking = false;                    // status of the braking
    int active_brightness;                              // brightness of the brake, based on how hard you brake
    int flashCount = 0;                                 // amount of times that the leds need to flash will be reduced by one each time by FlashRedLEDs
    int numActiveLEDs = 0;                              // number of LEDs from the center that are ON, based on how hard you brake

    bool flashlight = false;

    bool brakeWireInput = false;                        // status of the brake based on wire input

    Brake(Signals *signal, 
          CRGB *leds, 
          Gyro *gyro, 
          Button *button, 
          int numLEDs);
          
    void update();                                      // update the brake lights. numActiveLEDs and active_brightness must be changed externally (main)
    void flashRedLEDs();                                // flash led strip red based on flashCount
    void chromaMode();
};
