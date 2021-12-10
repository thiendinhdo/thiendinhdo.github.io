#include "mbed.h"
#include "Motor.h"
#include "PinDetect.h"
#include "rtos.h"
#include "ultrasonic.h"


Serial blue(p28,p27);       //UART
Motor right(p21, p6, p5);  // pwm, fwd, rev
Motor left(p22, p8, p7);   // pwm, fwd, rev

DigitalOut led1(LED1);

DigitalOut led2(LED2);
DigitalOut led3(LED3);

DigitalOut bluetoothMode(p15);
DigitalOut gestureMode(p16);

PinDetect pb(p10);

Serial pc(USBTX, USBRX);

DigitalOut warningLED(p19);
DigitalOut alarmLED(p20);
PwmOut speaker(p23);

Thread thread1;

//negative is fwd for the car

union f_or_char {
    float f;
    char  c[4];
};

bool isBluetooth = true;
bool isGesture = false;

void pb_hit_callback (void) {
    bluetoothMode = !bluetoothMode;
    gestureMode = !gestureMode;
    
    isBluetooth = !isBluetooth;
    isGesture = !isGesture;  
    
    if (isBluetooth)
    {
        led2 = 1;
    }
    else
    {
        led2 = 0;
    }
    
    if (isGesture)
    {
        led3 = 1;
    }
    else
    {
        led3 = 0;
    }
        
        
}
  
void uart_thread()
{
    char bchecksum=0;
    char temp=0;
    union f_or_char x,y,z;
   
    char bnum=0;
    char bhit=0;
    
    while(1) {
        
        if (isGesture)
        {
            bchecksum=0;
            if (blue.getc()=='!') {
                if (blue.getc()=='A') { //Accelerometer data packet
                    for (int i=0; i<4; i++) {
                        temp = blue.getc();
                        x.c[i] = temp;
                        bchecksum = bchecksum + temp;
                    }
                    for (int i=0; i<4; i++) {
                        temp = blue.getc();
                        y.c[i] = temp;
                        bchecksum = bchecksum + temp;
                    }
                    for (int i=0; i<4; i++) {
                        temp = blue.getc();
                        z.c[i] = temp;
                        bchecksum = bchecksum + temp;
                    }
                    if (blue.getc()==char(~('!' + 'A' + bchecksum))) { //checksum OK?
                        pc.printf("X = %f  Y = %f  Z = %f\n\r",x.f, y.f, z.f);
                        
                        if ((abs(x.f) < 0.25) && (abs(y.f) < 0.25))
                        {
                            pc.printf("STOP\r\n");
                            left.speed(0);
                            right.speed(0);
                        }
                        else if (x.f >= 0.25)
                        {
                            pc.printf("RIGHT\r\n");
                            left.speed(-0.5);
                            right.speed(0);
                        }
                        else if (x.f <= -0.25)
                        {
                            pc.printf("LEFT\r\n");
                            left.speed(0);
                            right.speed(-0.5);
                        }
                        else if (y.f >= 0.25)
                        {
                            pc.printf("FORWARD\r\n");
                            left.speed(-0.5);
                            right.speed(-0.5);
                        }
                        else if (y.f <= -0.25)
                        {
                            pc.printf("BACKWARD\r\n");
                            left.speed(0.5);
                            right.speed(0.5);
                        }
 
                    }
                }
            }
        }
        
        if (isBluetooth)
        {

            if (blue.getc()=='!') {
                if (blue.getc()=='B') { //button data packet
                    bnum = blue.getc(); //button number
                    bhit = blue.getc(); //1=hitt, 0=release
                    if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                        switch (bnum) {
                            case '5': //button 5 up arrow
                                if (bhit=='1') {
                                    pc.printf("FORWARD\r\n");
                                    left.speed(-0.5);
                                    right.speed(-0.5);
                                } else {
                                    pc.printf("STOP\r\n");
                                    right.speed(0);
                                    left.speed(0);
                                }
                                break;
                            case '6': //button 6 down arrow
                                if (bhit=='1') {
                                    pc.printf("BACKWARD\r\n");
                                    left.speed(0.5);
                                    right.speed(0.5);
                                } else {
                                    pc.printf("STOP\r\n");
                                    right.speed(0);
                                    left.speed(0);
                                }
                                break;
                            case '7': //button 7 left arrow
                                if (bhit=='1') {
                                    pc.printf("LEFT\r\n");
                                    left.speed(0);
                                    right.speed(-0.5);
                                } else {
                                    pc.printf("STOP\r\n");
                                    right.speed(0);
                                    left.speed(0);
                                }
                                break;
                            case '8': //button 8 right arrow
                                if (bhit=='1') {
                                    pc.printf("RIGHT\r\n");
                                    left.speed(-0.5);
                                    right.speed(0);
                                } else {
                                    pc.printf("STOP\r\n");
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
}

void dist(int distance)
{
    //put code here to execute when the distance has changed
    //pc.printf("Distance %d mm\r\n", distance);
    float note = 0;
    switch (distance/100) {
        case 0:
            note = (1.0f/440.0f);
            //speaker.period(note);
            //speaker = 0.1;
            alarmLED = 1;
            warningLED = 0;
            break;
        case 1:
            note = (1.0f/260.0f);
            //speaker.period(note);
            //speaker = 0.1;
            alarmLED = 0;
            warningLED = 1;
            break;
        default:
            note = 0;
            //speaker.period(note);
            //speaker = 0;
            alarmLED = 0;
            warningLED = 0;
            break;
    }
    if (note == 0)
        speaker = 0;
    else { 
        speaker.period(note);
        speaker = 0.1;
    }
   
}

ultrasonic mu(p17, p18, .1, 1, &dist);  //Set the trigger pin to p17 and the echo pin to p18
                                        //have updates every .1 seconds and a timeout after 1
                                        //second, and call dist when the distance changes
  
int main()
{
    bluetoothMode = 1;
    gestureMode = 0;
    
    
    led1 = 1;
    led2 = 1;
    led3 = 0;
    
    pb.mode(PullUp);
    pb.attach_deasserted(&pb_hit_callback);
    pb.setSampleFrequency();

    alarmLED = 1;
    warningLED = 1;
    wait(3);
    alarmLED = 0;
    warningLED = 0;
    
    thread1.start(uart_thread);
    
    // while(1)
    // {
    //     led1 = !led1;
    //     wait(0.5);
    // }

    mu.startUpdates();//start measuring the distance
    while(1)
    {
        //Do something else here
        mu.checkDistance();     //call checkDistance() as much as possible, as this is where
                                //the class checks if dist needs to be called.
        Thread::wait(100);  // delay by .1 seconds
    }
    
    
} 
