# Remote Controlled and Gesture Controlled  Car

## ECE 4180 Fall 2021 Final Project

##### Thien Dinh-Do, tdinhdo28@gatech.edu
##### George Madathany, gmadathany3@gatech.edu
##### Dilip Paruchuri, dparuchuri6@gatech.edu
##### Andrew Rocco, arocco3@gatech.edu


## Table of Contents

1. [Introdction](#introduction)
2. [Parts Used](#parts-used)
3. [Assembly, Pinouts, and Implimentation](#assembly-pinouts-and-implimentation)
   1. [Bluetooth Module](#bluetooth-module)
   2. [Motor Module](#motor-module)
   3. [Speaker Module](#speaker-module)
4. [Code](#code)
5. [Demo](#demo)


## Introduction

> We intend to develop a robotic car that can be operated in two ways: **remote control** and **gesture control**. The remote control will be through an iPhone app that will be transmitting data wirelessly to the mbed on the robotic car. We will be using the control pad of the iphone app to send the following commands: `forward`, `reverse`, `right`, `left`, and `stop`. The gesture control will also be established in the same iPhone app. The iPhone app is able to take accelerometer data measurements, and we will be sending that data to the robotic car through bluetooth, and based on the change in accelerometer data, the car will move. An additional feature we will add is crash detection. The robotic car wil sound an alarm and light and LED when it gets too close to an object. An additional feature we will add is by using a raspberry pi and a raspberry pi camera, we will add OpenCV object detection as the robotic car is controlled.


## Parts Used

- [mbed NXP LPC1768 Microcontroller (Cortex-M3)](https://www.sparkfun.com/products/9564)
    > The Mbed is a platform for IoT (Internet of Things) devices. It contains a 32-bit ARM core, and is similar to an arduino microcontroller. Mbed contains an online operating system and compiler that lets users program and create embedded systems projects. Users can interface various I/O peripherals to the Mbed. The input and outputs can fall in wide range such as digital inputs, digital outputs, analog inputs, analog outputs, and pulse width modulation (PWM) signals.
- [Shadow Chassis](https://www.sparkfun.com/products/13301)
    > The Shadow Chassis is a robotic car frame that comes with the Mbed parts kit. Georgia Tech students are required to purchase two parts kits throughout their curriculum: a basic parts kit for ECE 2035 and an extension parts kit for ECE 4180. The extension pack includes the shadow chassis. The shadow chassis when put together can support two motors connected to wheels, battery pack, microcontroller, and a breadboard.
- [Hobby Gearmotor - 140 RPM (Pair)](https://www.sparkfun.com/products/13302)
    > The gearmotors are utilized to turn the wheels of the RC car. The gear ratio of the motors are 48:1 which means they provide a large mechanical advantage. The Mbed alone cannot be directly interfaced with the motors. The mbed will be interfaced to the motors through a Dual H-Bridge which will allow the mbed to control both the speed and direction of both motors at the same time.
- [Wheel - 65mm (Rubber Tire, Pair)](https://www.sparkfun.com/products/13259)
- [Motor Driver - Dual TB6612FNG](https://www.sparkfun.com/products/14450)
    > A motor driver or Dual H-bridge is a chip that allow for a microcontroller like the mbed to control both the speed and direction of a motor. Normally, a microcontroller can not do that, but the h-bridge contains a fwd and rev signal for each motor along with a PWM signal to set the speed. More importantly this motor driver can be interfaced with two motors.
- [Adafruit Bluefruit LE UART Friend - Bluetooth Low Energy (BLE)](https://www.adafruit.com/product/2479)
    > The bluetooth module is utilized for wireless communication and control of the RC car utilizing the Adafruit app from the app store. In this project, the app will be used for the control pad which will allow for the user to control the RC car with the standard forward, right, left, back, and stop buttons. Also, the app will be used for its accelerometer readings which will be transmitted to the mbed for the gesture control aspect of the project.
- [Mono Audio Amp Breakout - TPA2005D1](https://www.sparkfun.com/)
    > The Class D Audio Amplifier is also an interface chip that is connected between the mbed and a speaker. The amplifier allows for a lower voltage graw as it only takes 3.3V for power. Moreover, it offers extra control over the speaker such as volume adjustment.
- [Speaker - PCB Mount](https://www.sparkfun.com/products/11089)
    > The speaker is an output device that has the ability to play wav files as well as audio samples loaded into the mbed. The speaker is relatively simple to use, and can be used without the amplifier, however it is best, when connected to the amplifier. The speaker is one of the components that is best used through and interrupt function in the code.
- [Ultrasonic Distance Sencsor - HC-SR04](https://www.sparkfun.com/products/15569)
    > This sensor measures the distance from the face of the sensor to the nearest object. The sensor is able to pick up objects up to 1 m away. It works by sending and sending out a sound wave and measures the distance from the wave that returns. It is not the most accurate sensor, however for the purposes of the project, it suffices.
- [LED - Basic Color 5mm](https://www.sparkfun.com/products/9590)
    > LEDs are one of the most simple yet versatile components that is available for embedded systems design. It can be utilized for quick testing and error checking, as well as in the case of the project, as a signal to indicate a message.
- [Wall Adapter Power Supply - 5VDC 2A](https://www.sparkfun.com/products/15312)
- [DC Barrel Jack Adapter](https://www.sparkfun.com/products/10811)


## Assembly, Pinouts, and Implimentation

Below are the important modules and their respective pinouts to the fucntionality of this project. Each are followed by a template or basic implimentation in `C++` code:

#### Bluetooth Module

![](ble.jpg)

| mbed | Adafruit LE UART BLE |
| :---: | :---: |
| GND | CTS |
| p27 (Serial RX) | TXO |
| p28 (Serial TX) | RXI |
| Vu (5V) | Vin |
| GND | GND |

<!-- mbed	Adafruit BLE
gnd	gnd
VU(5v)	Vin (3.3-16V)
nc	RTS
Gnd	CTS
p27 (Serial RX)	TXO
p28 (Serial TX)	RXI -->

[Code](https://os.mbed.com/users/4180_1/notebook/adafruit-bluefruit-le-uart-friend---bluetooth-low-/) below is useful for **remote control mode**:
```c++
#include "mbed.h"
BusOut myled(LED1,LED2,LED3,LED4);
Serial blue(p28,p27);
int main()
{
    char bnum=0;
    char bhit=0;
    while(1) {
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data packet
                bnum = blue.getc(); //button number
                bhit = blue.getc(); //1=hit, 0=release
                if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    myled = bnum - '0'; //current button number will appear on LEDs
                    switch (bnum) {
                        case '1': //number button 1
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '3': //number button 3
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '4': //number button 4
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '6': //button 6 down arrow
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '7': //button 7 left arrow
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}
```

#### Motor Module

![](hbridge.jpg)

| mbed | H-Bridge Motor Driver | Motor |
| :---: | :---: | :---: |
| Vu (5V) | VM |  |
| Vout | Vcc |  |
| GND | GND |  |
|  | A01 | R+ |
|  | A02 | R- |
|  | B02 | L- |
|  | B01 | L+ |
| GND | GND |  |
| p21 | PWMA |  |
| p5 | AI2 |  |
| p6 | AI1 |  |
| Vout | STBY |  |
| p7 | BI1 |  |
| p8 | BI2 |  |
| p22 | PWMB |  |
| GND | GND |  |


#### Speaker Module

![](classdbreakout.jpg)

| mbed | Class D Audio Amp TOA2005D1 | Speaker |
| :---: | :---: | :---: |
|  | OUT+ | + |
|  | OUT- | - |
| GND | PWR- |  |
| Vu (5V) | PWR+ |  |
|  | S |  |
| p23 | IN+ |  |
| GND | IN- |  |
| p18 | VOL |  |

<!-- mbed	TPA2005D1	Speaker
gnd	pwr - (gnd), in -	
Vout (3.3V) or 5V	pwr +	
p26 (any PWM or D/A)	in +	
out +	+
out -	-	
Any DigitalOut px(optional)	S (low for shutdown)	 -->

[Code](https://os.mbed.com/users/4180_1/notebook/using-a-speaker-for-audio-output/) below is useful for speaker interrupts:
```c++
#include "mbed.h"
// Audio output demo for speaker
// generates a 500Hz sine wave on the analog output pin
// 128 data points on one sine wave cycle are precomputed,
// scaled, stored in an array and
// continuously output to the Digital to Analog convertor
 
AnalogOut DAC(p18);
//global variables used by interrupt routine
volatile int i=0;
float Analog_out_data[128];
 
// Interrupt routine
// used to output next analog sample whenever a timer interrupt occurs
void Sample_timer_interrupt(void)
{
    // send next analog sample out to D to A
    DAC = Analog_out_data[i];
    // increment pointer and wrap around back to 0 at 128
    i = (i+1) & 0x07F;
}
 
int main()
{
    // set up a timer to be used for sample rate interrupts
    Ticker Sample_Period;
    // precompute 128 sample points on one sine wave cycle 
    // used for continuous sine wave output later
    for(int k=0; k<128; k++) {
        Analog_out_data[k]=((1.0 + sin((float(k)/128.0*6.28318530717959)))/2.0);
        // scale the sine wave from 0.0 to 1.0 - as needed for AnalogOut arg 
    }
    // turn on timer interrupts to start sine wave output
    // sample rate is 500Hz with 128 samples per cycle on sine wave
    Sample_Period.attach(&Sample_timer_interrupt, 1.0/(500.0*128));
    // everything else needed is already being done by the interrupt routine
    while(1) {}
}
```


## Code 

```c++
#include "mbed.h"
#include <stdio.h>
#include "Motor.h"
#include "rtos.h"
 
// Bluetooth and Motor pins
BusOut myled(LED1,LED2,LED3,LED4);
Serial blue(p28,p27);       // UART
Motor right(p21, p6, p5);   // pwm, fwd, rev
Motor left(p22, p8, p7);    // pwm, fwd, rev
 
//Sonar and speaker pins
DigitalOut trigger(p17);
DigitalIn echo(p18);
int correction = 0;
Timer sonar;                // for sonar distance measurement
 
// Warning LEDs when the robot is close to an object
DigitalOut warningLED(p19);
DigitalOut alarmLED(p20);
bool warning;               // true if the robot is close to an object
PwmOut speaker(p23);
volatile int i=0;           // for iterating through speaker data array
float Analog_out_data[128]; // data array for speaker sound
 
Thread thread;
// Interrupt routine
// used to output next analog sample whenever a timer interrupt occurs
void Sample_timer_interrupt(void)
{
    if (warning) {  // if in a warning state, send next analog sample out to D to A
        speaker = Analog_out_data[i];
    } else {        // if not in a warning state, turn speaker off
        speaker = 0;
    }
    // increment pointer and wrap around back to 0 at 128
    i = (i+1) & 0x07F;
}
 
void btMotor_thread() {
    char bnum=0;
    char bhit=0;
    while(1) {
        if (blue.getc()=='!') {
            if (blue.getc()=='B') {     // button data packet
                bnum = blue.getc();     // button number
                bhit = blue.getc();     // 1=hit, 0=release
                if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    myled = bnum - '0';     // current button number will appear on LEDs
                    switch (bnum) {
                        case '5': // button 5 up arrow
                            if (bhit=='1') {
                                left.speed(-0.5);
                                right.speed(-0.5);
                            } else {
                                right.speed(0);
                                left.speed(0);
                            }
                            break;
                        case '6':   //button 6 down arrow
                            if (bhit=='1') {
                                left.speed(0.5);
                                right.speed(0.5);
                            } else {
                                right.speed(0);
                                left.speed(0);
                            }
                            break;
                        case '7': //button 7 left arrow
                            if (bhit=='1') {
                                left.speed(0);
                                right.speed(-0.5);
                            } else {
                                right.speed(0);
                                left.speed(0);
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                left.speed(-0.5);
                                right.speed(0);
                            } else {
                                right.speed(0);
                                left.speed(0);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}
 
int main() {
    warning = false; // no warning at startup
    alarmLED = 0;
    warningLED = 0;
   
    // Sonar initialization
    int distance = 0;
    sonar.reset();  // reset sonar timer
    sonar.start();  // start timer for time correction
    while(echo==2){};
    sonar.stop();   // stop timer
    correction = sonar.read_us();   // read timer
   
    // Warning speaker initialization
    speaker.period(1.0/200000.0);   // set speaker period
    Ticker Sample_Period;   // ticker for speaker sounds and smaple rate
    for(int k=0; k<128; k++) {      // creates array for pwm output
        Analog_out_data[k]=((1.0 + sin((float(k)/128.0*6.28318530717959)))/2.0);
        // scale the sine wave from 0.0 to 1.0 - as needed for AnalogOut arg
    }
    Sample_Period.attach(&Sample_timer_interrupt, 1.0/(128*250.0));     // attach interrupt for speaker sampling
    thread.start(btMotor_thread);   // start bluetooth and motor control thread
 
    // Main thread for sonar, warning LEDs, and warning speaker
    while (1) {
        trigger = 1;    // trigger sonar to send a ping
        sonar.reset();
        wait_us(10.0);
        trigger = 0;
        while(echo==0){};   // wait for echo high
        sonar.start();      // echo high, start timer
        while(echo==1){};   // wait for echo low
        sonar.stop();       // echo low, stop timer
        distance = (sonar.read_us()-correction)/58.0;   // read value and calculate distance
        // When distance is below 50cm, initiate alarm
        // by turning on the alarm LED and playing the alarm sound (change frequency)
        if (distance < 50) {  
            warning = true;
            alarmLED = 1;
            warningLED = 0;
        // When distance is above 50cm but below 150cm, initiate warning
        // by turning on the warning and playing the warning sound (change frequency)
        } else if (distance < 150) {
            warning = true;
            alarmLED = 0;
            warningLED = 1;
        // When distance is above 150cm, uninitialize the warning/alarm
        // by turning off the warning/alarm and turning the speaker off
        } else {
            warning = false;
            alarmLED = 0;
            warningLED = 0;
        }
        Thread::wait(200);  // delay by .2 seconds
    }
}
 
```

## Demo

Click [here]() for the demo video.









<!-- You can use the [editor on GitHub](https://github.com/thiendinhdo/thiendinhdo.github.io/edit/main/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [Basic writing and formatting syntax](https://docs.github.com/en/github/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/thiendinhdo/thiendinhdo.github.io/settings/pages). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://docs.github.com/categories/github-pages-basics/) or [contact support](https://support.github.com/contact) and we’ll help you sort it out. -->
