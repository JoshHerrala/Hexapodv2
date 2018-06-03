/* This code drives a hexapod with 2 DOF legs and a SRF04 ultrasonic range finder. 

It was written for an Arduino Mega, and utilizes an Adafruit 16 channel servo driver (see text below)
and an SRF04 ultrasonic range finder. 

Written by Josh Herrala. www.networkoracle.com 

BSD license, all text above must be included in any redistribution.

*/

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


//******Rotation servo matches leg number**********
//Legs are numbered 1-6 starting with front left, 1-3 on left side, 4-6 on right. 4 is right front. 
//Triangle Left
int leglift1 = 0;
int leglift3 = 3;
int leglift5 = 4;
int legrot1 = 1;
int legrot3 = 2; //425bb servo
int legrot5 = 5;

//Triangle Right

int leglift4 = 6;
int leglift6 = 8;
int leglift2 = 14;
int legrot4 = 7;
int legrot6 = 9;
int legrot2 = 13;

//head rotation servo #
int headrot = 15;

int led = 13;
/*Initial leg position variables. Variations in servo type and mechanical differences
require each servo to be tuned to an initial neutral position.*/
int frontleftliftpos = 375;
int frontleftrotpos = 470; //-- makes it go forward more
int midleftliftpos = 500; //425bb servo
int midleftrotpos = 350;
int backleftliftpos  = 400;
int backleftrotpos = 500;

int frontrightliftpos = 400;
int frontrightrotpos = 450;
int midrightliftpos = 500;
int midrightrotpos = 350; 
int backrightliftpos = 400;
int backrightrotpos = 450;

int headposition = 300;

//Variables used to control walking speed.
int liftdelay = 1;
int rotdelay = 5;

//Variables used as counters in Do While loops that create walking motion.
int counter = 200;
int rotcounter = 200;
int wavecounter = 200;
int turncount = 0;
//Following variables define the send/receive pins for the SRF04. 
const int triggerPin = 22; //pin to send pulse
const int echoPin = 31;  //pin to receive pulse

//variables to sum distances using ping()
long pingone;
long pingtwo;
long pingsumone;
long pingsumtwo;


void setup() {
Serial.begin(9600);
pinMode(led, OUTPUT);
pwm.begin();
pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}


/*Main loop centers on using the SRF04 to determine if there are obstacles
in front of the robot, and take avoidance actions if required.*/
void loop()
{

if (ping() > 10) //call the range finder
{
   
   walk(); 
}
 else
 
 
 
 scan();

}

void walk() {
  //IMPORTANT. reducing the rotation range (rotcounter) requires repositioning starting positions. 
  walkpos();
  servoreset();
  
  leftlift();
  locomotion(0);
  //rightbackleftforward(); 
  dropleft();
  liftright();
  locomotion(1);
  //rightforwardleftback();
  dropright();
  }

 void leftturn(){
  scanpos(); 
  leftlift();
  locomotion(2);
  //rightbackleftforward(); 
  dropleft();
  liftright();
  locomotion(3);
  //rightforwardleftback();
  dropright();
 }
 
 void rightturn(){
   right_turn_pos();
   leftlift();
   locomotion(3);
   dropleft();
   liftright();
   locomotion(2);
   dropright();
 }



void scan() {
  scanpos();
  servoreset();
  
  
   headrotate(100);
   delay(100);
   pingone = ping();
   delay(500);
   headrotate(200);
   delay(100);
   pingtwo = ping();
   delay(500);
   pingsumone = (pingone + pingtwo);
   Serial.print(pingsumone + "right!");
   Serial.println();
   headrotate(445);
   delay(100);
   pingone = ping();
   delay(500);
   headrotate(555);
   delay(100);
   pingtwo = ping();
   delay(500);
   pingsumtwo = (pingone + pingtwo);
   Serial.print(pingsumtwo + " left!");
   Serial.println();
   headrotate(headposition);
   if (pingsumone < pingsumtwo)
   {
   Serial.print("turning right");
   Serial.println();
     
     do {
     rightturn();
     turncount++;
       } while(turncount <= 1);
   turncount = 0;
   }
   
   else 
   {
    Serial.print("turning left");
   Serial.println();
     
     do {
     leftturn();
     turncount++;
    } while(turncount <= 1);
   turncount = 0;
    }
 
 delay(1000); 
 headrotate(headposition);
/*
do {
//headrotate(320);
servofrontleftlift(frontleftliftpos);
servofrontrightlift(frontrightliftpos);  
frontleftliftpos--;
frontrightliftpos++;
wavecounter++;
  delay(liftdelay);
} while (wavecounter <= 300);

do {
//headrotate(280);
servofrontleftlift(frontleftliftpos);
servofrontrightlift(frontrightliftpos);
frontleftliftpos++;
frontrightliftpos--;
wavecounter--;
  delay(liftdelay);
} while (wavecounter >= 200);
*/

}

//servo moves*********************************************  
  
  void locomotion(int dir)
 {
   
 //0=right back, left forward 1=right forward, left back
 //2,3=turns
 
 do {
   servorightfrontrot(frontrightrotpos);
   servorightbackrot(backrightrotpos);
   servoleftmidrot(midleftrotpos);
   if (dir == 0)
   {
    frontrightrotpos--;
    midleftrotpos++;  
    backrightrotpos--;
   }
   else if (dir == 1)
   {
    frontrightrotpos++;
    midleftrotpos--; 
    backrightrotpos++;
   }
   else if (dir == 2)
   {
   frontrightrotpos--;
    midleftrotpos--;  
    backrightrotpos--;
   }
   else if (dir == 3)
   {
    frontrightrotpos++;
    midleftrotpos++; 
    backrightrotpos++;
   }
   
   servoleftfrontrot(frontleftrotpos);
   servorightmidrot(midrightrotpos);
   servobackleftrot(backleftrotpos);
   if (dir == 0)
   {
   frontleftrotpos--;
   midrightrotpos++;
   backleftrotpos--;
   }
   else if (dir == 1)
   {
   frontleftrotpos++;
   midrightrotpos--;  
   backleftrotpos++;
   }
   else if (dir == 2)
   {
   frontleftrotpos++;
   midrightrotpos++;
   backleftrotpos++;
   }
   else if (dir == 3)
   {
   frontleftrotpos--;
   midrightrotpos--;  
   backleftrotpos--;
   }
   
   rotcounter++;
   delay(rotdelay);
 } while (rotcounter <=300);
   rotcounter = 200;
 }
  
  void leftlift()
 {
  Serial.print("Lift Left");
  Serial.println();
  //Lift left triangle. This includes the front and rear legs on
  //on the left side, and the middle leg on the right side.
  do {
  servofrontleftlift(frontleftliftpos);
  servobackleftlift(backleftliftpos);
  servomidrightlift(midrightliftpos);
  midrightliftpos++;
  frontleftliftpos--;
  backleftliftpos--;
  counter++;
  delay(liftdelay);
} while (counter <= 300);
  counter = 200;
 }
 
 /*
 void rightbackleftforward()
 {
   Serial.print("Rotate Right Triangle Back, Left Forward.");
 Serial.println();
 //Right triangle legs move back, pushing robot forward. Left legs
 //move forward.
 do {
   servorightfrontrot(frontrightrotpos);
   servorightbackrot(backrightrotpos);
   servoleftmidrot(midleftrotpos);
   frontrightrotpos--;
   midleftrotpos++;  
   backrightrotpos--;
   servoleftfrontrot(frontleftrotpos);
   servorightmidrot(midrightrotpos);
   servobackleftrot(backleftrotpos);
   frontleftrotpos--;
   backleftrotpos--;
   midrightrotpos++; 
   
   
   rotcounter++;
   delay(rotdelay);
 } while (rotcounter <=300);
   rotcounter = 200;
 }
*/

void dropleft()
{
  Serial.print("Drop Left");
 Serial.println();
 //Drop left triangle legs
 do {
  servofrontleftlift(frontleftliftpos);
  servomidrightlift(midrightliftpos);
  servobackleftlift(backleftliftpos);
  midrightliftpos--;
  frontleftliftpos++;
  backleftliftpos++;
  counter++;
  delay(liftdelay);
  
} while (counter <= 300); 
  counter = 200;
}

void liftright()
{
  Serial.print("Lift Right");
Serial.println();
//Lift right triangle legs
  do {
  servofrontrightlift(frontrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackrightlift(backrightliftpos);
  midleftliftpos--;
  backrightliftpos++;
  frontrightliftpos++;
  counter++;
  delay(liftdelay);
 
} while (counter <= 300);
  counter = 200; 
}

/*
void rightforwardleftback()
{
  Serial.print("Rotate Right Forward, Left Backward");
Serial.println();
//Rotate right triangle forward, left triangle legs move backward
//pushing the robot forward.
 do {
   servorightfrontrot(frontrightrotpos);
   servorightbackrot(backrightrotpos);
   servoleftmidrot(midleftrotpos);
   frontrightrotpos++;
   backrightrotpos++;
   midleftrotpos--;  
   servoleftfrontrot(frontleftrotpos);
   servorightmidrot(midrightrotpos);
   servobackleftrot(backleftrotpos);
   frontleftrotpos++;
   midrightrotpos--;  
   backleftrotpos++;
  
   rotcounter++;
   delay(rotdelay);
 } while (rotcounter <= 300);
   rotcounter = 200;
}

*/

void dropright()
{
  Serial.print("Drop Right");
Serial.println();
//Drop right triangle
  do {
  servofrontrightlift(frontrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackrightlift(backrightliftpos);
  backrightliftpos--;
  midleftliftpos++;
  frontrightliftpos--;
  counter++;
  delay(liftdelay);
  
} while (counter <= 300);
  counter = 200;
}

 void servoreset()
 {
  //reset the left
  servofrontleftlift(frontleftliftpos);
  servoleftfrontrot(frontleftrotpos);
  servomidleftlift(midleftliftpos);
  servoleftmidrot(midleftrotpos);
  servobackleftlift(backleftliftpos);
  servobackleftrot(backleftrotpos);
  //reset the right
  servofrontrightlift(frontrightliftpos);
  servorightfrontrot(frontrightrotpos);
  servomidrightlift(midrightliftpos);
  servorightmidrot(midrightrotpos);
  servobackrightlift(backrightliftpos);
  servorightbackrot(backrightrotpos);
   
  headrotate(headposition);
  

}

/*Servo positioning functions. These functions are called with each
pass through the do while loops above. */
//****************************************************************** 

void headrotate(int pulselen) //head position
{
  
  pwm.setPWM(headrot, 0, pulselen);
 Serial.print("headcalled");  
  Serial.println();
}


void servofrontleftlift(int pulselen) //Lift position of leg 1, front left lift
{
   pwm.setPWM(leglift1, 0, pulselen);
   
  }

void servobackleftlift(int pulselen) //Lift position of leg 3, back left lift
{
   
   pwm.setPWM(leglift3, 0, pulselen); 
   }


void servomidrightlift(int pulselen) //Lift position of leg 5, middle right lift
  {
    pwm.setPWM(leglift5, 0, pulselen);
  
}
  
void servofrontrightlift(int pulselen) //Lift position of leg 4, front right lift
{
   pwm.setPWM(leglift4, 0, pulselen);
   
   }
  
  void servobackrightlift(int pulselen) //Lift position of leg 6, back right lift
{
   
  pwm.setPWM(leglift6, 0, pulselen); 
   }
   
 void servomidleftlift(int pulselen)  //Lift position of leg 2, middle left lift
{
    pwm.setPWM(leglift2, 0, pulselen);
  }
  
  void servorightfrontrot(int pulselen) //Rotation position of leg 4, front right rotation
  {
    pwm.setPWM(legrot4, 0, pulselen);
   
  }
  
  void servorightbackrot(int pulselen) //Rotation position of leg 6, back right rotation
  {
    
   pwm.setPWM(legrot6, 0, pulselen);
  }
  
  void servoleftmidrot(int pulselen) //Rotation position of leg 2, middle left rotation
  {
    pwm.setPWM(legrot2, 0, pulselen); 
  } 
    
    void servoleftfrontrot(int pulselen) //Rotation position of leg 1, front left rotation
    {
    pwm.setPWM(legrot1, 0, pulselen);
     
    }
   
   void servobackleftrot(int pulselen) //Rotation position of leg 3, back left rotation
    {
    pwm.setPWM(legrot3, 0, pulselen);
    }
   
   
   void servorightmidrot(int pulselen) //Rotation position of leg 5, middle right rotation
  {
    pwm.setPWM(legrot5, 0, pulselen);  
  }  
  
  //sonic range finder functions**************************************
  
  long ping()
  {
    // establish variables for duration of the ping,
// and the distance result in centimeters:
long duration, cm;

// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
pinMode(triggerPin, OUTPUT);
digitalWrite(triggerPin, LOW);
delayMicroseconds(2);
digitalWrite(triggerPin, HIGH);
delayMicroseconds(5);
digitalWrite(triggerPin, LOW);

// The echo pin is used to read the signal from the PING))): a HIGH
// pulse whose duration is the time (in microseconds) from the sending
// of the ping to the reception of its echo off of an object.
pinMode(echoPin, INPUT);
duration = pulseIn(echoPin, HIGH);

// convert the time into a distance
cm = microsecondsToCentimeters(duration);
//Use code below to see distance using serial monitors
Serial.print(cm);
Serial.print("cm");
Serial.println();
return cm;  
}
  
  long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

 //servo default position functions

 void scanpos()
 {
   /*Initial leg position variables. Variations in servo type and mechanical differences
  require each servo to be tuned to an initial neutral position.*/
  frontleftliftpos = 380;
  frontleftrotpos = 400; //-- makes it go forward more
  midleftliftpos = 530; //425bb servo
  midleftrotpos = 390;
  backleftliftpos  = 410;
  backleftrotpos = 465;

  frontrightliftpos = 400;
  frontrightrotpos = 400;
  midrightliftpos = 400;
  midrightrotpos = 400; 
  backrightliftpos = 400;
  backrightrotpos = 400;

  headposition = 300;
 }
 
 void walkpos()
 {
  /*Initial leg position variables. Variations in servo type and mechanical differences
  require each servo to be tuned to an initial neutral position.*/
  frontleftliftpos = 375;
  frontleftrotpos = 470; //-- makes it go forward more
  midleftliftpos = 500; //425bb servo
  midleftrotpos = 350;
  backleftliftpos  = 400;
  backleftrotpos = 500;

  frontrightliftpos = 400;
  frontrightrotpos = 450;
  midrightliftpos = 400;
  midrightrotpos = 350; 
  backrightliftpos = 400;
  backrightrotpos = 450;

  headposition = 300;
 }
 
 void right_turn_pos()
 {
  /*Initial leg position variables. Variations in servo type and mechanical differences
  require each servo to be tuned to an initial neutral position.*/
  frontleftliftpos = 375;
  frontleftrotpos = 350; //-- makes it go forward more
  midleftliftpos = 500; //425bb servo
  midleftrotpos = 300;
  backleftliftpos  = 400;
  backleftrotpos = 420;

  frontrightliftpos = 400;
  frontrightrotpos = 350;
  midrightliftpos = 400;
  midrightrotpos = 450; 
  backrightliftpos = 400;
  backrightrotpos = 350;

  headposition = 300;
 }
 
 //testing loops using the leds
 
 void fastled(){
   int ledcount = 1;
   
   do{
     digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100); 
  ledcount++;
  } while(ledcount <=  30);
  ledcount = 1;
  delay(500);
   }
   
    

