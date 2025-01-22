# Motorcycle-Signals
As we know, rear-ends, while not the most fatal, are considered to be the most common type of collision. For motorcyclist, unfortunetly, this puts them in an impossible situation, that sometimes leads to their deaths.
For this reason, I created this open-source Arduino project, focused on increasing road safety for motorcyclists by making them more visible from behind, as well as better communicate their intentions on roadways.

## The Problems
Motorcyclists know that there are two fundamental problems with their brake lights:
1. The first is that they are far too small, making it hard to judge their distance, sometimes going completely unnoticed. This is one of the problems that this projects aims to fix, by incorporating a center flashing element, which is sure to get drivers attention.
2. The second, and arguably more dangerous problem, is that the rear light is activated solely based on brake input. This means that if the motorcyclist chooses to downshift instead of using their brakes, the brake lights will never turn on. This varies from bike to bike, but regardless, it would be important to communicate this information to the drivers behind us, as all they care about is whether were slowing down or not, regardless of brake activation.
These two effects are hightened even more when we consider the weight of motorcycles, sometimes cutting their braking distances by half in ideal circumstances. 

## The Solution
Thus, this project will have a few set goals:
1. Use a gyroscope to detect braking instead of brake input (bumps will have to be filtered out).
3. Lightly flash the center of the brake light when braking.
4. Flash the entire brake light when emergency braking is detected.
5. Flash the entire brake light when braking has been initialize (when braking for the first time).
6. The brake light will light up MORE the harder the deceleration (*Ex; at -1m/s^2 have 10 LEDs turn on, when -2m/s^2 have 20 LEDs turn on, etc*).
7. Incorporate sequential turn signals.

## Circuit Diagram
![Screenshot 2025-01-22 014738](https://github.com/user-attachments/assets/1307465a-f116-4e59-b317-fbbc9e9f7539)
> *Two DC-DC Buck converters are used, one to drive the 12V LEDs and one to drive the 5V Arduino Nano. However, to be safe, the Buck converters are set to 11V and 4.5V respectively.*
>
> *Additionally, at full brightness, the WS2815s and Buck may run hot: Each LED can draw 13mA, at 12V thats 0.16W per LED. With 66 LEDs, thats 11W.*
>
> *Note that some LED strips only have one data pin, while some like the WS2815s have a backup one. D4 is the data pin by default, but it can be changed within the `main` headers
> 
> *For this reason, try to keep the current draw below 0.75A. If the LEDs or Buck are getting hot, simply lower the brightness of the LEDs.*

![Screenshot 2025-01-22 014846](https://github.com/user-attachments/assets/78e6ea16-7ef2-4a84-8516-3350251b6d51)

## Customization and Header Files
I did my best to make this project as versatile and scalable as possible. Therefore, the LEDs should be able to be tailored and customized easily, by simply changing the parameters in the header files.
To do this, after downloading the program files, simply open all the files ending in `.h` and change the values of the `#define`'s to suit your liking.
![Screenshot 2025-01-05 165439](https://github.com/user-attachments/assets/3c05c40d-e91a-444e-a3c6-27e4b3e6d679)

## Demos
https://github.com/user-attachments/assets/aa2e7507-03d2-4963-8b66-d6189a29fe5b
> Gyroscope filtering algorithm, to remove any noise and bumps that would skew the readings of the gyroscope, and therefore light up the LED strip incorectly

https://github.com/user-attachments/assets/a8786f55-660a-4991-8c82-4a94363032eb
> Demo of signal/hazard lights
