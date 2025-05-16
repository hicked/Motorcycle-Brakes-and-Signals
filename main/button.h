#pragma once
#include <Arduino.h>

#define BUTTON_PIN 8 // Pin for the button
#define BUTTON_ENABLED true

#define DEFAULT_MODE BRAKE_MODE_DYNAMIC // pick from the enum below

enum BrakeMode {
    BRAKE_MODE_DYNAMIC, // Brake lights are on based on how hard you brake
    BRAKE_MODE_STATIC, // Brake lights are all on when braking
    
    BRAKE_MODE_HAZARDS,
    BRAKE_MODE_CHROMA, // Chroma mode
    BRAKE_MODE_CHRISTMAS, // Christmas mode
    BRAKE_MODE_HALLOWEEN, // Halloween mode
    BRAKE_MODE_FLASHLIGHT, // Flashlight mode
    
    BRAKE_MODE_NUM_MODES // Placeholder DO NOT REMOVE, SHOULD ALWAYS BE LAST
};

class Button {
public:
    bool state = false;
    bool prevState = false;
    int mode = DEFAULT_MODE;
    void update();
};