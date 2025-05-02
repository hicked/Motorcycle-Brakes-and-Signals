#include "Brake.h"

Brake::Brake(Signals *signal, CRGB *leds, Gyro *gyro, Button *button, int numLEDs) {
    this->LEDStrip = leds;
    this->numLEDs = numLEDs;
    this->signals = signal;
    this->gyro = gyro;
    this->button = button;
    this->middleIndex = this->numLEDs / 2;
    fill_solid(this->LEDStrip, this->numLEDs, CRGB::Black);
}

void Brake::update() {
    unsigned long currentTime = millis();
    this->brakeWireInput = !digitalRead(BRAKE_PIN);

    // Emergency braking detected
    if (gyro->smoothedAndCorrectedYAcc < -MAX_GYRO_BREAKING && flashCount == 0) {
        flashCount++; // continuously adds one or flashes forever
    }

    // Initialization of braking detected
    else if (gyro->smoothedAndCorrectedYAcc < -MIN_GYRO_BREAKING && gyro->prevSmoothedAndCorrectedYAcc >= -MIN_GYRO_BREAKING && 
            millis() - this->timeSinceLastIniBraking > TIME_BETWEEN_INI_BRAKE) {
        timeSinceLastIniBraking = millis();
        flashCount = INITIALIZE_BRAKING_FLASH_LENGTH;
    }

    switch (this->button->mode) {
        case BRAKE_MODE_DYNAMIC:
            this->dynamicBrakeMode();
            break;
        case BRAKE_MODE_HAZARDS:
            this->dynamicBrakeMode();
            // Signals handled by signals class
            break;
        case BRAKE_MODE_CHROMA:
            this->chromaMode();
            break;
        case BRAKE_MODE_CHRISTMAS:
            this->christmasMode();
            break;
        case BRAKE_MODE_HALLOWEEN:
            this->halloweenMode();
            break;
        case BRAKE_MODE_FLASHLIGHT:
            this->flashlightMode();
            break;
        case BRAKE_MODE_STATIC:
            this->staticBrakeMode();
            break;
        default:
            // Default to dynamic brake mode
            this->dynamicBrakeMode();
            break;
    }
}



void Brake::dynamicBrakeMode() {
    this->setSolid(BACKLIGHT_COLOUR); // Reset to background
    if (gyro->smoothedAndCorrectedYAcc < -MIN_GYRO_BREAKING) { // breaking
        // Flash the LEDs in the center
        this->flashCenterLEDs();

        // If signals on, use static braking (not progressive)
        if (signals->left || signals->right) {
            for (int i = 0; i < numLEDs/2 - CENTER_FLASH_WIDTH/2 - SIGNAL_LENGTH; i++) {
                int index1 = middleIndex + CENTER_FLASH_WIDTH/2 + i;
                int index2 = middleIndex - CENTER_FLASH_WIDTH/2 - i - 1; 

                this->LEDStrip[index1] = ACTIVE_BRAKE_COLOUR;
                this->LEDStrip[index2] = ACTIVE_BRAKE_COLOUR;
            }         
        } 
        // Otherwise, use progressive brake lighting
        else {
            for (int i = 0; i < this->numActiveLEDs; i++) {
                int index1 = middleIndex + CENTER_FLASH_WIDTH/2 + i;
                int index2 = middleIndex - CENTER_FLASH_WIDTH/2 - i - 1; 

                this->LEDStrip[index1] = ACTIVE_BRAKE_COLOUR; // red brake color
                this->LEDStrip[index2] = ACTIVE_BRAKE_COLOUR;
            }
        }
    } 
    
    else if (gyro->smoothedAndCorrectedYAcc > MIN_GYRO_ACCELERATING && SHOW_ACC) { // Accelerating
        for (int i = 0; i < this->numActiveLEDs; i++) {
            int index1 = middleIndex + i;
            int index2 = middleIndex - i - 1;
            
            this->LEDStrip[index1] = ACTIVE_ACC_COLOUR; // Green brake color
            this->LEDStrip[index2] = ACTIVE_ACC_COLOUR;
        }
    }
    // if (millis() > this->cac+250) {
    //     int numSpaces = 33-this->numActiveLEDs-CENTER_FLASH_WIDTH;
    //     for (int i=0; i<numSpaces; i++){
    //         Serial.print("-");
    //     }
    //     for (int i=0; i<this->numActiveLEDs; i++){
    //         Serial.print("=");
    //     }
    //     for (int i=0; i<CENTER_FLASH_WIDTH; i++) {
    //         Serial.print("+");
    //     }
    //     for (int i=0; i<this->numActiveLEDs; i++){
    //         Serial.print("=");
    //     }
    //     for (int i=0; i<numSpaces; i++){
    //         Serial.print("-");
    //     }
    //     Serial.println("");
    //     cac = millis();
    // }
}

void Brake::staticBrakeMode(){
    this->setSolid(BACKLIGHT_COLOUR); // Reset to background
    if (gyro->smoothedAndCorrectedYAcc < -MIN_GYRO_BREAKING || this->brakeWireInput) {
        this->flashCenterLEDs();
        for (int i = 0; i < numLEDs/2 - CENTER_FLASH_WIDTH/2; i++) {
            int index1 = middleIndex + CENTER_FLASH_WIDTH/2 + i;
            int index2 = middleIndex - CENTER_FLASH_WIDTH/2 - i - 1; 

            this->LEDStrip[index1] = ACTIVE_BRAKE_COLOUR;
            this->LEDStrip[index2] = ACTIVE_BRAKE_COLOUR;
        }        
    }
    else if (gyro->smoothedAndCorrectedYAcc > MIN_GYRO_ACCELERATING && SHOW_ACC) { // accelerating
        this->setSolid(ACTIVE_ACC_COLOUR); // Green brake color
    }
}

void Brake::flashCenterLEDs() { // Flashing of the center LEDs
    unsigned long currentTime = millis();
    // If the time since the last flash is greater than the delay betweem flashes
    if (currentTime - this->lastCenterFlashTime >= CENTER_FLASH_SPEED) {
        this->lastCenterFlashTime = currentTime;
        this->centerFlashON = !this->centerFlashON;
    }

    for(int i = 0; i < CENTER_FLASH_WIDTH/2; i++) {
        int index1 = middleIndex + i;
        int index2 = middleIndex - i - 1;

        if (this->centerFlashON) {
            this->LEDStrip[index1] = CENTER_FLASH_COLOUR; // Flash red
            this->LEDStrip[index2] = CENTER_FLASH_COLOUR;
        }
    }
}

void Brake::flashRedLEDs() { // Flashing of the entire LED strip
    unsigned long currentTime = millis();

    if (currentTime - this->lastFlashTime >= FLASH_SPEED) {
        this->lastFlashTime = currentTime;
        this->flashON = !this->flashON;

        if (this->flashON) {
            this->setSolid(FLASH_COLOUR); // Flash red
            this->flashCount--;
        } else {
            this->setSolid(BACKLIGHT_COLOUR); // Reset to background
        }
    }
}

void Brake::setSolid(CRGB colour) {
    fill_solid(this->LEDStrip, this->numLEDs, colour);
}


// Different brake light patterns/modes
void Brake::chromaMode() {
    CRGB colours[] = {CRGB::Red, CRGB::Green, CRGB::Blue}; // List of colors
    marqueeEffect(colours, 3, 30, 0.85f, true, false);
}

void Brake::christmasMode() {
    CRGB colours[] = { CRGB::Blue, CRGB::Purple, CRGB::Red, CRGB::Orange, CRGB::White, CRGB::Green };
    shiftPatternMode(colours, 6, 30, 15, false); // Speed of 30ms, size of 10
}

void Brake::halloweenMode() {
    CRGB colours[] = { CRGB::Purple, CRGB(255, 65, 0), CRGB::Black };
    marqueeEffect(colours, 3, 30, 0.85f, true, false);
}

void Brake::flashlightMode() {
    fill_solid(this->LEDStrip, this->numLEDs, FLASHLIGHT_COLOUR);
}


/**
 *
 * Functions used to define the different brake light patterns/modes that are above
 *
**/
void Brake::shiftPatternMode(CRGB colors[], int numColors, unsigned long speed, int size, bool invertDirection=false) {
    unsigned long currentTime = millis();

    // Check if enough time has passed to update the pattern
    if (currentTime - this->lastPatternTime >= speed) {
        this->lastPatternTime = currentTime;

        static int shift = 0; // Tracks the current shift of the pattern

        for (int i = 0; i < this->numLEDs; i++) {
            // Calculate the effective position within the pattern
            int effectiveIndex;
            if (invertDirection) {
                // Invert the direction: shift is subtracted instead of added
                effectiveIndex = (i + shift) % (numColors * size);
            } else {
                // Normal direction: shift is added
                effectiveIndex = (i - shift + (numColors * size)) % (numColors * size);  // Add offset to prevent negative index
            }

            // Determine the color for the current LED based on the effective position
            int segment = effectiveIndex / size; // Which segment (color) this LED belongs to

            // Set the color for the current LED
            this->LEDStrip[i] = colors[segment];
        }

        // Increment the shift, ensuring it wraps around properly after one full cycle
        shift = (shift + 1) % (numColors * size);
    }
}


void Brake::marqueeEffect(CRGB* colours, int numColours, int speed, float blendFactor, bool fromCenter=true, bool invertDirection=false) {
    static uint16_t offset = 0; // Keeps track of the current offset
    unsigned long currentTime = millis();

    if (currentTime - this->lastPatternTime >= speed) {
        this->lastPatternTime = currentTime;

        if (!fromCenter) {
            // Regular marquee: moves left to right or right to left
            for (int i = 0; i < this->numLEDs; i++) {
                // Calculate the effective position within the pattern
                int effectiveIndex = (i - (invertDirection ? -offset : offset)) % this->numLEDs;

                // Determine the color for the current LED based on the effective position
                int currentColour = effectiveIndex / (this->numLEDs / numColours); // Which segment (color) this LED belongs to

                // Blend the two colors together based on the blend factor
                this->LEDStrip[i] = this->blendColours(this->LEDStrip[i], colours[(currentColour + 1) % numColours], blendFactor);
            }
        } else {
            // Marquee from center to outwards
            for (int i = 0; i < this->numLEDs / 2; i++) {
                // Calculate the distance from the center for the left and right LEDs
                int index1 = middleIndex + i;
                int index2 = middleIndex - i - 1;

                // Determine which color to use for the current LED based on the distance
                int currentColourIndex = (i - (invertDirection ? -offset : offset)) % this->numLEDs / (this->numLEDs / numColours);
                int nextColourIndex = (currentColourIndex + 1) % numColours;


                // Apply the blended color to both the left and right LEDs
                this->LEDStrip[index1] = this->blendColours(this->LEDStrip[index1], colours[nextColourIndex], blendFactor);
                this->LEDStrip[index2] = this->blendColours(this->LEDStrip[index2], colours[nextColourIndex], blendFactor);
            }
        }
        // Increment the offset to extend the pattern outward symmetrically from the center
        offset++;
    }
}

CRGB Brake::blendColours(CRGB colour1, CRGB colour2, float blendFactor) {
    return CRGB
    (
        blendFactor * colour1.r + (1-blendFactor) * colour2.r,
        blendFactor * colour1.g + (1-blendFactor) * colour2.g,
        blendFactor * colour1.b + (1-blendFactor) * colour2.b
    );
}