// Used Arduino Nano board, with old bootloader

#include <FastLED.h> // 3.9.9
#include "brake.h"
#include "signals.h"
#include "gyro.h"
#include "button.h"

#define LED_DATA_PIN 4 // CHANGE TO 5 IF DATA PIN NOT WORKING

#define NUM_LEDS 66
#define LED_TYPE WS2815

CRGB leds[NUM_LEDS];

Gyro *gyro;
Brake *brake;
Button *button;
Signals *signals;

void setup() {
    FastLED.addLeds<LED_TYPE, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
    
    FastLED.clear();
    FastLED.setBrightness(GLOBAL_BRIGHTNESS);

    // Initialize Serial for debugging
    Serial.begin(115200);

    // Setup pins
    // MAY NEED TO CHANGE THESE TO REGULAR INPUTS LATER
    pinMode(LEFT_SIGNAL_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SIGNAL_PIN, INPUT_PULLUP);
    pinMode(BRAKE_PIN, INPUT_PULLUP);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize objects
    button = new Button();
    gyro = new Gyro(button);
    signals = new Signals(leds, button, NUM_LEDS);
    brake = new Brake(signals, leds, gyro, button, NUM_LEDS);
    Serial.println("Setup and Calibration Complete");
}


void loop() {
    gyro->update(); // Updates the value of smoothedAcc
    button->update(); // Updates the button mode

    // Serial.println(button->mode);
    // Serial.println(gyro->smoothedAcc);

    // Update the number of active LEDs and brightness of the brake lights
    if (gyro->smoothedAndCorrectedYAcc > 0) { // accelerating
        brake->numActiveLEDs = constrain(map(gyro->smoothedAndCorrectedYAcc, MIN_GYRO_ACCELERATING, MAX_GYRO_ACCELERATING, 0, NUM_LEDS/2), 0, NUM_LEDS/2);
        brake->active_brightness = constrain(map(gyro->smoothedAndCorrectedYAcc, MIN_GYRO_ACCELERATING, MAX_GYRO_ACCELERATING, MIN_BRAKE_BRIGHTNESS, MAX_BRAKE_BRIGHTNESS), 0, MAX_BRAKE_BRIGHTNESS);
    } else { // breaking
        brake->numActiveLEDs = constrain(map(abs(gyro->smoothedAndCorrectedYAcc), MIN_GYRO_BREAKING, MAX_GYRO_BREAKING, 0,  (NUM_LEDS - CENTER_FLASH_WIDTH)/2), 0, (NUM_LEDS - CENTER_FLASH_WIDTH)/2);
        brake->active_brightness = constrain(map(-(gyro->smoothedAndCorrectedYAcc), MIN_GYRO_BREAKING, MAX_GYRO_BREAKING, MIN_BRAKE_BRIGHTNESS, MAX_BRAKE_BRIGHTNESS), 0, MAX_BRAKE_BRIGHTNESS);
    }

    if (abs(gyro->smoothedAndCorrectedYAcc - EXPECTED_ACC_MAGNITUDE) < 1000 && brake->brakeWireInput) { // at rest, brake activated
        brake->numActiveLEDs = NUM_LEDS/2-1;
    }

    if (SHOW_CHROMA && gyro->smoothedAndCorrectedYAcc > MAX_GYRO_ACCELERATING) {
        CRGB colours[] = {CRGB::Red, CRGB::Green, CRGB::Blue};
         // (list of colours, number of colours, speed, blend amount, start from center, reverse direction)
        brake->chromaMode();

        signals->update(); // signals on top of mario star
    }
    else if (brake->flashCount == 0) {
        brake->update();
        signals->update(); // signals on top of brake lighting
    }  
    else { // handled outside brake.update() since we want it to go OVER the turn signals, while normal brake lights go UNDER signals
        brake->flashRedLEDs();
    }
    
    FastLED.show();
}