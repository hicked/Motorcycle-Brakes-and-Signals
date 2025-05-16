#include "signals.h"

Signals::Signals(CRGB *leds, Button *button, int num_leds) {
    this->LEDStrip = leds;
    this->numLEDs = num_leds;
    this->button = button;
}

void Signals::update() {
    unsigned long currentTime = millis();

    this->updateStates();
    if (this->numActiveLEDs < SIGNAL_LENGTH && currentTime - lastSignalUpdate > SIGNAL_SPEED && (this->left || this->right)) {
        lastSignalUpdate = currentTime;
        this->numActiveLEDs++;
    }
    // pauses when bar is full momentarily
    else if (this->numActiveLEDs >= SIGNAL_LENGTH && currentTime - lastSignalUpdate > PAUSE_TIME) {
        lastSignalUpdate = currentTime;
        this->numActiveLEDs = 0;
    }

    if (left) {
        leftSignal();
    } 
    if (right) {
        rightSignal();
    }
    if (!right && !left) {
        this->numActiveLEDs = 0;
    }
}

void Signals::updateStates() {
    // Left turn signal
    if (!digitalRead(LEFT_SIGNAL_PIN)) {
        this->left = true;
        leftLastHighTime = millis();
    } else if (this->left && (millis() - leftLastHighTime >= SIGNAL_OFF_DELAY)) {
        this->left = false;
    }

    // Right turn signal
    if (!digitalRead(RIGHT_SIGNAL_PIN)) {
        this->right = true;
        rightLastHighTime = millis();
    } else if (this->right && (millis() - rightLastHighTime >= SIGNAL_OFF_DELAY)) {
        this->right = false;
    }

    if (this->button->mode == BRAKE_MODE_HAZARDS) {
        this->left = true;
        this->right = true;
    }
}

void Signals::rightSignal() {
    int startIndex = numLEDs / 2 + (numLEDs / 2 - SIGNAL_LENGTH);

    for (int i = 0; i < numActiveLEDs; i++) {
        FastLED.setBrightness(255);
        LEDStrip[startIndex+i] = SIGNAL_COLOUR; 
    }
}

void Signals::leftSignal() {
    int startIndex = numLEDs / 2 - (numLEDs / 2 - SIGNAL_LENGTH) - 1;

    for (int i = 0; i < numActiveLEDs; i++) {
        FastLED.setBrightness(255);
        LEDStrip[startIndex-i] = SIGNAL_COLOUR; 
    }
}