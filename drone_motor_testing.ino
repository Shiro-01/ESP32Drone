/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.14.08 or later version;
     - for iOS 1.11.2 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    
#include <Servo.h>  // Include Servo library for PWM control

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT


// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "shiro"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

//testing

#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 54 bytes
  { 255,5,0,0,0,47,0,18,0,0,0,31,1,106,200,1,1,3,0,5,
  12,18,49,49,32,2,26,31,5,15,121,49,49,32,2,26,31,2,56,79,
  44,22,1,2,26,31,31,79,78,0,79,70,70,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t joystick_02_x; // from -100 to 100
  int8_t joystick_02_y; // from -100 to 100
  uint8_t switch_01; // =1 if switch ON and =0 if OFF

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


Servo esc1;  // Create servo object for ESC1
Servo esc2;  // Create servo object for ESC2
Servo esc3;  // Create servo object for ESC2
Servo esc4;  // Create servo object for ESC2

void setup() 
{
  RemoteXY_Init (); 
    pinMode(5, OUTPUT); // Set pin 5 as an output

esc1.attach(9);  // Attach the first ESC to pin 9

esc2.attach(10); // Attach the second ESC to pin 10
esc3.attach(8); // Attach the second ESC to pin 10
esc4.attach(7); // Attach the second ESC to pin 10
  //esc.writeMicroseconds(1000);  // Send the minimum throttle signal to the ESC
  // Wait 2 seconds to allow the ESC to recognize the startup condition
  delay(2000); 

  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  
    digitalWrite(5, RemoteXY.switch_01);  // Turn the LED off

  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay(), use instead RemoteXY_delay() 

   int throttle1 = map(max(0, RemoteXY.joystick_01_x), 0, 100, 1000, 2000);  // ESC1 using Joystick 1 X-axis // throtle 1 for just going up
  int throttle2 = map(max(0, RemoteXY.joystick_02_x), 0, 100, 1000, 2000);  // ESC2 using Joystick 2 X-axis
  
  // Send throttle signals to ESCs
  esc1.writeMicroseconds(throttle1);
  esc2.writeMicroseconds(throttle1);
  esc3.writeMicroseconds(throttle1);
  esc4.writeMicroseconds(throttle1);


}
