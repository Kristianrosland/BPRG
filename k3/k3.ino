#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>
 
//Pins
int laserPin = 6;
int fanPin = 7;
int powerButtonPin = 8;
int bottomServoPin = 9;
int topServoPin = 10;
int buzzer = 13;
 
//Servo stuff
Servo bottomServo;
Servo topServo;
int MIN_ANGLE_TOP_SERVO = 760;
int MAX_ANGLE_TOP_SERVO = 1350;
int MIN_ANGLE_BOTTOM_SERVO = 1100;
int MAX_ANGLE_BOTTOM_SERVO = 1300;
int TOP_SERVO_MIDDLE = 985;
int BOTTOM_SERVO_MIDDLE = 1200;
int bottomServoMs;
int topServoMs;
 
//Bluetooth stuff
SoftwareSerial btSerial(12, 11, false);
int val;
 
//Distance stuff
SoftwareSerial tfSerial(4, 5);
int dist;  //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
int check;  //save check value
int i;
int uart[9];  //save data measured by LiDAR
const int HEADER=0x59;  //frame header of data package
 
void setup() {
  Serial.begin(115200);
  btSerial.begin(9600);
  //tfSerial.begin(115200);
  //tfSerial.listen();

  //Fan and button setup
  pinMode(powerButtonPin, INPUT);
  pinMode(laserPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(laserPin, LOW);
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH);
 
  //Servo setup
  rotateDirectly(BOTTOM_SERVO_MIDDLE, TOP_SERVO_MIDDLE);
  bottomServo.attach(bottomServoPin);
  topServo.attach(topServoPin);
 
  tone(buzzer, 1000); 
  delay(200);      
  noTone(buzzer); 
}
 
void loop() {
  int buttonState = digitalRead(powerButtonPin);
  if (buttonState == 1) {
    autoFire();
  }
 
  if (btSerial.available()) {
    val = btSerial.read();
    if (val == 'z') {
      int newX = readTripleDigit();
      int newY = readTripleDigit();
      //rotateDirectly(newX, newY);
    }
    else if (val == 'f') {
      fire();
    } else if (val == 'a') {
      autoFire();
    } else {
      //rotate(val);
    }
  }
}

void autoFire() {
  laser(true);
  
  int x_closest_ms;
  int distance;
  findClosestCup(&x_closest_ms, &distance);

  laser(false);
  
  fireOnTarget(x_closest_ms, distance);
}

void findClosestCup(int* x_closest_ms, int* shortest_distance) {
  int distance = 0;
  int strength = 0;
  int minMs=MIN_ANGLE_BOTTOM_SERVO;
  int maxMs=MAX_ANGLE_BOTTOM_SERVO;
  int scanHeight=TOP_SERVO_MIDDLE - 50;
  int bestX = -1;
  int shortest = 1000;

  //Get in position
  rotate(maxMs, scanHeight);
  delay(200);

  //Scan to find closest cup
  for (int ms = maxMs; ms >= minMs; ms--) {
    rotate(ms, scanHeight);
    delay(50);
    
    getDistance(&distance, &strength);
    if (distance < shortest) { 
      Serial.println("New shortest, " + String(ms) + ": " + String(distance));
      shortest = distance;
      bestX = ms;
    }
  }

  *x_closest_ms = bestX;
  *shortest_distance = shortest;
}

void fireOnTarget(int x, int dist) {
  int y = calculateFireAngle(dist);
  boolean success = rotate(x, y);
  if (success) {
    countdown(3);
    fire(); 
  }
}

int calculateFireAngle(int distance) {
  return MAX_ANGLE_TOP_SERVO; //TODO
}

boolean rotate(const int xMs, const int yMs) {
  boolean success = true;
  while (!(bottomServoMs == xMs && topServoMs == yMs) && success) {
    if (bottomServoMs != xMs) {
      bottomServoMs += (xMs - bottomServoMs) / abs(xMs - bottomServoMs); // +-1 in the right direction
    }
    if (topServoMs != yMs) {
      topServoMs += (yMs - topServoMs) / abs(yMs - topServoMs); // +-1 in the right direction
    }
    
    //Rotate and keep successvalue
    success &= setBottomServo(bottomServoMs);
    success &= setTopServo(topServoMs);
    delay(5);
  }
  return success;
}
 
void rotateDirectly(const int xMs, const int yMs) {
  int delayTime = max(abs(bottomServoMs-xMs), abs(topServoMs-yMs)) * 5;

  setBottomServo(xMs);
  setTopServo(yMs);
  delay(delayTime);
}

boolean setTopServo(const int ms) {
  if (ms < MIN_ANGLE_TOP_SERVO || ms > MAX_ANGLE_TOP_SERVO) { Serial.println("Illegal rotation, x = " + String(ms)); return false; }
  topServoMs = ms;
  topServo.writeMicroseconds(ms);
  return true;
}

boolean setBottomServo(const int ms) {
  if (ms < MIN_ANGLE_BOTTOM_SERVO || ms > MAX_ANGLE_BOTTOM_SERVO) { Serial.println("Illegal rotation, y = " + String(ms)); return false; }
  bottomServoMs = ms;
  bottomServo.writeMicroseconds(ms);
  return true;
}
 
void fire() {
  digitalWrite(fanPin, LOW);
  delay(500);
  digitalWrite(fanPin, HIGH);
}

void laser(boolean on) {
  digitalWrite(laserPin, on ? HIGH : LOW);
}

//Delay x seconds, playing the buzzer for each second
void countdown(int seconds) {
  for (int i = 1; i <= seconds; i++) {
    tone(buzzer, 1000); 
    delay(200 + (i == seconds ? 300 : 0));      
    noTone(buzzer); 
    delay(500);
  }
}

/** HELPER FUNCTIONS **/

//Used to read from bluetooth
int readTripleDigit() {
  int value = 0;
  value += 100 * readDigit();
  value += 10 * readDigit();
  value += 1 * readDigit();
 
  return value;
}
 
int readDigit() {
  while(!btSerial.available()) {}
  return btSerial.read()-48; //-48 to convert from ASCII
}

//Get distance from TF-mini
void getDistance(int* rDist, int* rStr) {
  unsigned int dist = 0;
  unsigned int str = 0;

  while(Serial.available() > 0) {
    char t = Serial.read();
  }  

  while(dist==0||dist==-1) {
        while(Serial.available()>=9)
        {
            if((0x59 == Serial.read()) && (0x59 == Serial.read())) //Byte1 & Byte2
            {
                unsigned int t1 = Serial.read(); //Byte3
                unsigned int t2 = Serial.read(); //Byte4
    
                t2 <<= 8;
                t2 += t1;
                dist=t2;
    
                t1 = Serial.read(); //Byte5
                t2 = Serial.read(); //Byte6
    
                t2 <<= 8;
                t2 += t1;
                str=t2;
    
                for(int i=0; i<3; i++) 
                { 
                    Serial.read(); ////Byte7,8,9
                }
            }
      }
  }
  *rDist=dist;
  *rStr=str;
}
