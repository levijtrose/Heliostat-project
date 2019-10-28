 
#include <Wire.h>
#include "RTClib.h"
#include <LSM303.h>
DS1307 rtc;
LSM303 compass;
const int sleepPinA = 2; //jump reset and sleepPins 
const int stepPinA = 3; //Motor A is for rotation, B is for tilting
const int dirPinA = 4;
const int sleepPinB = 5;
const int stepPinB = 6;
const int dirPinB = 7;
const int ledPin = 13;

char month[] = "JANFEBMARAPRMAYJUNJULAUGSEPOCTNOVDEC";
const float brislatitude = -27.48;
const float brislongitude = 153.01;
const int brismeridian = 150;
const int headingmax = 80;
const int headingmin = -80;
const int pitchmax = 45;
const int pitchmin = 0;

int rotateconst = 60;
int upconst = 90;
int downconst = 90;
int bisectheading;
int bisectpitch;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(sleepPinA, OUTPUT);
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinA, OUTPUT);
  pinMode(sleepPinB, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite (sleepPinA, LOW);
  digitalWrite (sleepPinB, LOW);
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-440 -650 -520}; //calibrated uniquely for this compass original values were -440 -650 -520
  compass.m_max = (LSM303::vector<int16_t>){+580 +540 +600}; //                                                          +580 +540 +600
  rtc.begin();
  // rtc.adjust(DateTime(__DATE__, __TIME__)); //Use this to set the RTC for the first time only
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
}

void loop() {
  digitalWrite (sleepPinA, LOW);
  digitalWrite (sleepPinB, LOW);
  delay (1000);
  DateTime now = rtc.now();
  if (! rtc.isrunning()) Serial.println ("RTC error");

    //FIRST CALCULATE ALL THIS CRAP

    /*Solar declination seems to calculate slightly wrong, difference between SOLAR TIME and LOCAL TIME
    not taken into consideration - this can account for upto 30 mins variance.*/ 

  int hourangle = (now.hour() * 15)+(now.minute() /4)-180;   //-180 to 180   
  int yearday = (((now.unixtime())/ 86400L)-17896);          //current day into this year 
  float equationoftime = 9.87 * cos(0.9863 * 2 * (yearday + 10)*DEG_TO_RAD) - 7.53 * sin(0.9863 * (yearday + 10)*DEG_TO_RAD) - 1.5 * sin(0.9863 * (yearday + 10)*DEG_TO_RAD);
  float timecorrection = (brislongitude - brismeridian) + equationoftime; //calculated in minute degrees
  int solartime = hourangle + timecorrection;                                      
  float sundeclination = -23.44 * cos(0.9863 * (yearday + 10)*DEG_TO_RAD);
  float sunaltitude = asin(sin(brislatitude*DEG_TO_RAD)*sin(sundeclination*DEG_TO_RAD) + cos(brislatitude*DEG_TO_RAD)*cos(sundeclination*DEG_TO_RAD)*cos((solartime)*DEG_TO_RAD))*RAD_TO_DEG; 
  float sunazimuth = asin((-sin((solartime)*DEG_TO_RAD)*cos(sundeclination*DEG_TO_RAD))/(cos(sunaltitude*DEG_TO_RAD)))*RAD_TO_DEG; 
  int heading = compassheading();
  int pitch = compasspitch();
  if (sunazimuth > 180) sunazimuth -= 360;
  
    // calculating target heading and bisecting angle 
    
  float targetheading = 45 + (10 * cos(now.minute() * 6 * DEG_TO_RAD));
  float targetpitch = -11.5 - (4.5 * sin(now.minute() * 6 * DEG_TO_RAD)); 
  bisectheading = (sunazimuth+targetheading)/2;  
  bisectpitch = (sunaltitude + targetpitch)/2;
  if (bisectpitch < 0) bisectpitch =0;
  
    // make sure rotating constants and bisect heading/pitch are within reasonable bounds
  if (rotateconst < 30 || rotateconst > 120) rotateconst = 60;
  if (upconst < 60 || upconst > 120) upconst = 90;
  if (downconst <60 || downconst > 120) downconst = 90;
  if (bisectheading > headingmax) bisectheading = headingmax;
  if (bisectheading < headingmin) bisectheading = headingmin;
  if (bisectpitch > pitchmax) bisectpitch = pitchmax;
  if (bisectpitch < pitchmin) bisectpitch = pitchmin;

    //SERIAL OUTPUT

  Serial.print ("date\t\t");
  Serial.print (now.year());
  Serial.print (" ");
    Serial.print (month[((now.month()-1)*3)]);
    Serial.print (month[((now.month()-1)*3+1)]);
    Serial.print (month[((now.month()-1)*3+2)]);
  Serial.print (" ");
  Serial.println (now.day());
  Serial.print ("time\t\t"); 
  Serial.print (now.hour());
  Serial.print (":");
  if (now.minute() <10) Serial.print ("0");
     Serial.println (now.minute());
  Serial.print ("Solar hourangle\t");
  Serial.println (solartime);
  Serial.print ("Sun Declination\t");
  Serial.println (sundeclination);
  Serial.print ("Sun Altitude\t");
  Serial.println (sunaltitude);
  Serial.print ("Sun azimuth\t");
  Serial.println (sunazimuth);
  Serial.print ("heading\t\t");
  Serial.println (heading);
  Serial.print ("pitch\t\t");
  Serial.println (pitch);
 /* Serial.print ("target heading\t");
  Serial.println (targetheading);
  Serial.print ("target pitch\t");
  Serial.println (targetpitch); */
  Serial.print ("Bisect heading\t");
  Serial.println (bisectheading);
  Serial.print ("Bisect pitch\t");
  Serial.println (bisectpitch);
  Serial.print ("Rotate constant\t");
  Serial.println (rotateconst);
  Serial.print ("Tiltup constant\t");
  Serial.println (upconst);
  Serial.print ("Tiltdn constant\t");
  Serial.println (downconst);
 
    //CHECK IF CURRENT HEADING/PITCH OR SUN/TIME ARE OUT OF OPERATING RANGE

  if (heading > (headingmax + 5) || heading < (headingmin -5)) {
    Serial.println ("Current heading out of range RESET MANUALLY");
    for (int x = 0;  x < 10 ; x ++) {
      digitalWrite (ledPin, HIGH);
      delay (500);
      digitalWrite (ledPin, LOW);
      delay (500);
    }
    return;
  }
  if (pitch > (pitchmax +5) || pitch < (pitchmin -5)) {
    Serial.println ("Current pitch  out of range RESET MANUALLY");
    for (int x = 0;  x < 10 ; x ++) {
      digitalWrite (ledPin, HIGH);
      delay (500);
      digitalWrite (ledPin, LOW);
      delay (500);
    }
    return;
  }
  if (sunaltitude < 30 || sunazimuth < 0 || sunazimuth > 120) {
    Serial.println ("Sun's altitude or azimuth out of range");
    bisectheading = 60;
    bisectpitch = 40;
  }
  if (now.hour() < 7 || now.hour() >= 12) {
    Serial.println ("Outside operating hours 6:00 - 12:00");
    bisectheading = 60;
    bisectpitch = 40;
  }

     // PUT TEST ANGLES HERE USING BISECTHEADING/PITCH


    // NORMAL ROTATING CONDITIONS is the heading 2 degrees off target and is the bisect/target in operable range
  
  if ((abs(bisectheading-heading) >= 3) && (headingmin <= bisectheading && bisectheading <= headingmax) && (rtc.isrunning() && compass.init())) { 
    rotate(bisectheading-heading);
    }
  
  if ((abs(bisectpitch-pitch)>= 3) && (pitchmin <= bisectpitch && bisectpitch <= pitchmax) && (rtc.isrunning() && compass.init())) {
    tilt(bisectpitch-pitch);
  }
    //Check if on target
  if ((abs(bisectheading-heading) < 3) && (abs(bisectpitch-pitch) < 3)) {
    Serial.println ("ON TARGET RESTING");
    delay(60000);
    return;
  }

  Serial.println ("RESTING");
  delay(1000);
}

  //ROTATE FUNCTION
  
void rotate(int degrees) {
  if (degrees > 0) {
    digitalWrite(sleepPinA, HIGH);
    digitalWrite(dirPinA, HIGH);
    delay(10);
    Serial.print ("ROTATING\t");
    Serial.println (degrees);
    for (int x = 0; x < (degrees * rotateconst); x ++) {
      digitalWrite (stepPinA, HIGH);
      delayMicroseconds (500);
      digitalWrite (stepPinA, LOW);
      delayMicroseconds (500);
    }
    digitalWrite(sleepPinA, LOW);
    delay(1000);
    if (bisectheading > compassheading()) rotateconst += 1;  //adjust rotational constant coefficient
    if (bisectheading < compassheading()) rotateconst -= 1;
  }
  if (degrees < 0) {
    digitalWrite(sleepPinA, HIGH);
    digitalWrite(dirPinA, LOW);
    delay(10);
    Serial.print ("ROTATING\t");
    Serial.println (degrees);
    for (int x = 0; x > (degrees * rotateconst); x --) {
      digitalWrite (stepPinA, HIGH);
      delayMicroseconds (500);
      digitalWrite (stepPinA, LOW);
      delayMicroseconds (500);
    }
    digitalWrite(sleepPinA, LOW);
    delay(1000);
    if (bisectheading > compassheading()) rotateconst -= 1;
    if (bisectheading < compassheading()) rotateconst += 1;
  } 
}

  // TILT FUNCTION
  
void tilt(int degrees) {
  if (degrees > 0) {
    digitalWrite(sleepPinB, HIGH);
    digitalWrite(dirPinB, HIGH);
    delay(10);
    Serial.print ("TILTING\t\t");
    Serial.println (degrees);
    for (int x = 0; x < (degrees * upconst); x ++) {
      digitalWrite (stepPinB, HIGH);
      delayMicroseconds (500);
      digitalWrite (stepPinB, LOW);
      delayMicroseconds (500);
    }
    digitalWrite(sleepPinB, LOW);
    delay(1000);
    if (bisectpitch > compasspitch()) upconst += 1;
    if (bisectpitch < compasspitch()) upconst -= 1;
  }
  if (degrees < 0) {
    digitalWrite(sleepPinB, HIGH);
    digitalWrite(dirPinB, LOW);
    delay(10);
    Serial.print ("TILTING\t\t");
    Serial.println (degrees);
    for (int x = 0; x > (degrees * downconst); x --) {
      digitalWrite (stepPinB, HIGH);
      delayMicroseconds (500);
      digitalWrite (stepPinB, LOW);
      delayMicroseconds (500);
    }
    digitalWrite(sleepPinB, LOW);
    delay(1000);
    if (bisectpitch > compasspitch()) downconst -= 1;
    if (bisectpitch < compasspitch()) downconst += 1;
  } 
}

// COMPASS FUNCTIONS

float compassheading() {
    if (compass.init()) compass.read ();
    else Serial.println ("Compass error");
    float x = compass.heading((LSM303::vector<int>){1, 0, 0});
    if (x > 180) x -= 360;
    return x;
  }

float compasspitch() {
  if (compass.init()) compass.read ();
  else Serial.println ("Compass error");
  float x = (atan(compass.a.x/sqrt(pow(compass.a.y,2) + pow(compass.a.z,2)))*RAD_TO_DEG);
  return x;
}

 /*Solar declination seems to calculate slightly wrong,

  */
