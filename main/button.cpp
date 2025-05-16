#include "Button.h"

void Button::update() {
    if (BUTTON_ENABLED) {
        this->state = !digitalRead(BUTTON_PIN);

        if (this->state && !this->prevState) { // button pressed
            this->state = true;
            this->mode = (this->mode+1)%BRAKE_MODE_NUM_MODES;
        }
        
        this->prevState = this->state;
    }
}