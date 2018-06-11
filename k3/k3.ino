#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>
 
//Pins
int potPin = 5;
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
int MAX_ANGLE_TOP_SERVO = 1380;
int MIN_ANGLE_BOTTOM_SERVO = 900;
int MAX_ANGLE_BOTTOM_SERVO = 1400;
int TOP_SERVO_MIDDLE = 985;
int BOTTOM_SERVO_MIDDLE = 1200;
int FEED_HEIGHT = 1250;
int SCAN_STOP_X = 1150;
int SCAN_START_X = 1250;
int SCAN_HEIGHT = 900;
int bottomServoMs;
int topServoMs;
int lastShotX;
int lastShotDist;

//margins
double LIDAR_MARGIN = 20.0;
double CUP_MARGIN = 3.0;
double LIDAR_ERROR = -13.0;
 
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
    if (buttonHold() || lastShotX == 0 || lastShotDist == 0) {
      autoFire();
    } else {
      refire();
    }
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

  if (distance < 100 || distance > 260) {
    playErrorSound();
    rotate(BOTTOM_SERVO_MIDDLE, TOP_SERVO_MIDDLE);
    return;
  }

  waitForBall();
  fireOnTarget(x_closest_ms, distance);
}

void refire() {
  fireOnTarget(lastShotX, lastShotDist);
}

void waitForBall() {
  rotate(MIN_ANGLE_BOTTOM_SERVO, FEED_HEIGHT);
  delay(3000);
}

void findClosestCup(int* x_closest_ms, int* shortest_distance) {
  int distance = 0;
  int strength = 0;
  int bestX = -1;
  int shortest = 1000;

  int potmeterVal = analogRead(potPin) / 10;
  int scan_height = SCAN_HEIGHT + 10 + potmeterVal;
  Serial.println("Scan height = " + String(SCAN_HEIGHT) + " + " + String(potmeterVal));
  
  //Get in position
  rotate(SCAN_START_X, scan_height);
  delay(200);

  //Scan to find closest cup
  for (int ms = SCAN_START_X; ms > SCAN_STOP_X; ms--) {
    rotate(ms, scan_height);
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
  lastShotX = x;
  lastShotDist = dist;
  int actualDistance = dist + LIDAR_MARGIN + CUP_MARGIN + LIDAR_ERROR;
  int y = calculateFireAngle(actualDistance);
  Serial.println("Measured distance: " + String(dist) + " (+ 3)");
  Serial.println("Calculated distance: " + String(actualDistance) + " (+ 3)");

  boolean success = rotate(x, y);
  if (success) {
    countdown(3);
    fire(); 
    delay(300);
    rotate(MIN_ANGLE_BOTTOM_SERVO, FEED_HEIGHT); 
  }
}

double calculateFireAngle(double distance) {
  //TEMP
  double y1 = calculateDistance(distance);
  Serial.println("Calculate fire angle: " + String(y1));
  int r = random(10);
  int RANDOM_MARGIN = r < 2 ? 20 : (r == 3 ? -10 : 0);
  if (y1 == 1380) { RANDOM_MARGIN = 0; }
  return min(y1 + RANDOM_MARGIN, 1380);
  //TEMP
  double x = distance;
  double y = 0.0005298497*pow(x,4) - 0.4685001313*pow(x,3) + 155.2950455278*pow(x,2) - 22867.6231360983*x + 1263164.3;
  Serial.println("Calculate fire angle: " + String(y));
  return min(y, MAX_ANGLE_TOP_SERVO);
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
    delay(2);
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

void playErrorSound() {
  tone(buzzer, 1000);
  delay(250);
  tone(buzzer, 800);
  delay(250);
  noTone(buzzer);
}

/** Using line segments between each measured point **/
double calculateDistance(double dist) {
  double distances[10] =    { 224.9, 229.5, 232.9, 234.6, 235.4, 237.1, 237.5, 241.9, 237.8, 227.5 };
  double microseconds[10] = { 1190,  1210,  1230, 1250, 1270, 1290, 1310, 1330, 1350, 1380 };
  int x = dist;
  if (x > 242) { return 1330; }
  for (int i = 8; i >= 0; i--) {
    if (x > distances[i] && x < distances[i+1]) {
       double diff = distances[i+1] - distances[i];
       double delta = x - distances[i];
       double percent = delta/diff;
       double deltaMs = (microseconds[i+1]-microseconds[i]) * percent;
       double y = microseconds[i] + deltaMs;
       return y;
    } else if (x < distances[i] && x > distances[i+1]){
      double diff = distances[i] - distances[i+1];
      double delta = abs(x - distances[i]);
      double percent = delta/diff;
      double deltaMs = abs(microseconds[i+1]-microseconds[i]) * percent;
      double y = microseconds[i] + deltaMs;
      return y;
    }
  }
  return microseconds[0];
}

boolean buttonHold() {
  for (int i = 0; i < 10; i++) {
    delay(100);
    if (digitalRead(powerButtonPin) == 0) {
      return false;
    }
  }
  return true;
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

//Used when measuring length/angle
void measure(int ms) {
  while (true) {
    int button = digitalRead(powerButtonPin);
    while (button == 0) { button = digitalRead(powerButtonPin); }
    boolean hold = true;
    for (int i = 0; i < 100 && hold; i++) {
      hold &= (digitalRead(powerButtonPin) == 1);
      delay(10);
    }

    if (hold) {
      ms += 10;
      tone(buzzer, 1000); 
      delay(200);      
      noTone(buzzer); 
      Serial.println("New ms: " + String(ms));
      rotate(MIN_ANGLE_BOTTOM_SERVO, ms);
      delay(1500);
    } else {
      rotate(BOTTOM_SERVO_MIDDLE, ms);
      delay(1500);
      fire();
      delay(1000);
      rotate(MIN_ANGLE_BOTTOM_SERVO, 1200);
    }
  }
}
