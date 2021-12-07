# Remote Controlled and Gesture Controlled  Car

## ECE 4180 Fall 2021 Final Project

##### Thien Dinh-Do, tdinhdo28@gatech.edu
##### George Madathany, gmadathany3@gatech.edu
##### Dilip Paruchuri, dparuchuri6@gatech.edu
##### Andrew Rocco, arocco3@gatech.edu


## Table of Contents

1. [Objective](#objective)
2. [Parts Used](#parts-used)
3. [Assembly and Pinouts](#assembly-and-pinouts)
4. [Code](#code)
5. [Demo](#demo)


## Objective


## Parts Used

- [mbed NXP LPC1768 Microcontroller (Cortex-M3)](https://www.sparkfun.com/products/9564)
- [Shadow Chassis](https://www.sparkfun.com/products/13301)
- [Hobby Gearmotor - 140 RPM (Pair)](https://www.sparkfun.com/products/13302)
- [Wheel - 65mm (Rubber Tire, Pair)](https://www.sparkfun.com/products/13259)
- [Motor Driver - Dual TB6612FNG](https://www.sparkfun.com/products/14450)
- [Adafruit Bluefruit LE UARD Friend - Vluetooth Low Energy (BLE)](https://www.adafruit.com/product/2479)
- [Mono Audio Amp Breakout - TPA2005D1](https://www.sparkfun.com/)
- [Speaker - PCB Mount](https://www.sparkfun.com/products/11089)
- [Ultrasonic Distance Sencsor - HC-SR04](https://www.sparkfun.com/products/15569)
- [LED - Basic Oragne 5mm](https://www.sparkfun.com/products/533)
- [LED - Basic Red 5mm](https://www.sparkfun.com/products/9590)
- [Wall Adapter Power Supply - 5VDC 2A](https://www.sparkfun.com/products/15312)
- [DC Barrel Jack Adapter](https://www.sparkfun.com/products/10811)


## Assembly and Pinouts

| mbed | herro |
| --- | ---|




## Code 

```
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

Click [here](<!--- inser link --->) for the demo video.









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
