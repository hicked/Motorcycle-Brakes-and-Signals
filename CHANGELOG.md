# CHANGELOG:

Date is in YYYY/MM/DD format.

**v.A.B.C**

**A**: Major Feature / Major Bug Fix

**B**: Minor Feature / Minor Bug Fix

**C**: Refactoring / Tiny Feature / Patch

## [v.2.1.1] - 2024-02-08
**Author:** Antoine
Improved bump detection
  - If bump is detected, uses raw y acceleration as value. This wont be acurate if on a hill, but is better than no info at all. **Might change this later**
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These may have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes

## [v.2.0.1] - 2024-02-06
**Author:** Antoine
- Fixed bug where calibration acceleration wouldnt be properly calibrate
- Fixed bug where it would infinitely calibrate when it can't read from mpu
changed headers for more realistic acc/braking
- If it fails to read MPU (gyro) during calibration, it will change the mode to static
- Improved bump detection
  - If bump is detected, uses raw y acceleration as value. This wont be acurate if on a hill, but is better than no info at all. **Might change this later**
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These may have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes

## [v.2.0.0] - 2024-01-29
**Author:** Antoine
-   Gyro filtering was deemed to be ineffective, resorting to Y-Axis acceleration with vector algebra to determine when lean/hills occur (and disregard them).
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These may have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes

## [v.1.0.0] - 2024-01-22
**Author:** Antoine
-   v1.0.0, First release
-   Refactoring
-   Gyro filter effectiveness to be determined
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These may have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes

## [v.0.4.1-beta] - 2024-01-18
**Author:** Antoine
-   Renamed versions, v1.0.0 should be out soon
-   Refactoring
-   Made general functions for LED patterns that can be adjusted manually
    -   May need some refactoring/polishing for `marquee`
-   Fixed dynamic brake mode not working
-   Gyro filter effectiveness to be determined
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These will have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes
> Also note that, for testing purposes, gyroscope is taking in raw input (gravity is not disregarded)

## [v.0.4.0-beta] - 2024-01-16
**Author:** Antoine
-   Refactoring
-   Added switch cases for button modes
    -   **Dynamic (lights up progressively)**
    -   **Static (ON/OFF)** <br><br>
  
    -   Christmas
    -   Halloween
    -   Chroma
    -   Flashlight
-   Added **bonus led data pin** in case `5` doesnt work use `4`
-   Default button mode also added, along with `BUTTON_ENABLED`
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These will have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes
> Also note that, for testing purposes, gyroscope is taking in raw input (gravity is not disregarded)

## [v.0.3.2-beta] - 2024-01-05
**Author:** Antoine
-   Finalized Signals
-   Added flashlight
-   Refactoring
-   Changed format to RGB
-   Included circuit diagram and other media
> Note, signal and brake inputs are set to `INPUT_PULLUP` for now to ignore noise. These will have to be swapped to `INPUT`. Signals input are also inverted for debugging purposes

## [v.0.3.1-alpha] - 2024-12-29
**Author:** Antoine
-   Fixed brake light flashing logic that wasnt working
-   Made it so the center leds always flash when braking
-   Made it so signals have variable length (header)
-   Made it so if signal is on, brake becomes static (solid, not progressive)
-   Implemented detecting of signal from wire input of bike
> Note all colours are in GRB for now since different leds are used

## [v.0.3.0-alpha] - 2024-12-26
**Author:** Antoine
-   Gyro logic, including filter, completed.
    -   Now ignores gravity (calculated during calibration phase on startup)
    -   filters out big spikes in gyro values
        -   if bumps are detected for an extending period of time, overrides
        -   if bumps seem to have a consistent acceleration value, overrides
    -   filters by taking an average from sample size
    -   filters by applying smoothing affect to acceleration
-   Gyro must how only be RELATIVELY flat. 
-   Gyro must do its calibration with no external acceleration present (not accelerating)
> Lots of parameters can be changed in the `gyro.h` header file to get more favorable results


## [v.0.2.0-prealpha] - 2024-11-30
**Author:** Antoine
-   **ALL CONTROL BASED ON GYRO AS INTENDED**
-   ADDED CONRTOLS BASED ON GYRO Y AXIS
-   MUST ENSURE THE GYRO LIGHT IS FACING UP, WITH THE PINS ON THE RIGHT SIDE
-   Issue with brake light, too sensitive, sometimes ini flash will be continuous
> ENSURE GYRO IS FLAT. IT WILL PRINT INITIAL VALUES TO LET YOU KNOW

## [v.0.1.0-prealpha] - 2024-11-28
**Author:** Antoine
-   **ALL CONTROL BASED ON ENCODER FOR TESTING PURPOSES**
-   Sequential brake lights work
-   Turn signals work
-   Sequention acceleration works, can be turned off in header
-   Mario Star mode for acceleration, can be turned off in header
-   Flash when initialize braking
-   Flash continuously when hard braking

https://github.com/user-attachments/assets/23fc0a4a-a3bb-4595-a7d4-98dc2ed54147

