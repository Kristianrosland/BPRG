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
int bottomServoAngle = 45;
int topServoAngle = 45;
int MIN_ANGLE_TOP_SERVO = 25;
int MAX_ANGLE_TOP_SERVO = 80;
int topAngleAddr = 0;
int bottomAngleAddr = 1;
int topServoHorizontalAngleMs = 970;
 
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
  topServoAngle = EEPROM.read(topAngleAddr);
  bottomServoAngle = EEPROM.read(bottomAngleAddr);
  if (topServoAngle == 0) {
    topServoAngle = (MIN_ANGLE_TOP_SERVO + MAX_ANGLE_TOP_SERVO) / 2;
  }
  if (bottomServoAngle == 0) {
    bottomServoAngle = 45;
  }

  topServo.write(topServoAngle);
  bottomServo.write(bottomServoAngle);

  bottomServo.attach(bottomServoPin);
  topServo.attach(topServoPin);
 
  //topServo.write(topServoAngle);
  //bottomServo.write(bottomServoAngle);
  
  delay(500);
  Serial.println("Ready");
  tone(buzzer, 1000); 
  delay(200);      
  noTone(buzzer); 
}
 
void loop() {
  int buttonState = digitalRead(powerButtonPin);
  if (buttonState == 1) {
    Serial.println("Click!");
    autoFire();
  }
 
  if (btSerial.available()) {
    val = btSerial.read();
    if (val == 'z') {
      int newX = readTripleDigit();
      int newY = readTripleDigit();
      rotateDirectly(newX, newY);
    }
    else if (val == 'f') {
      fire();
    } else if (val == 'a') {
      autoFire();
    } else {
      rotate(val);
    }
  }
  
}

float float2ms(float degrees)
{
  return topServoHorizontalAngleMs + degrees * 50.0/9.0;
}

unsigned int int2ms(unsigned int degrees)
{
  return  topServoHorizontalAngleMs + degrees * 150 / 27;
}

void setX(const int xVal) {
  bottomServo.write(bottomServoAngle);
  EEPROM.write(bottomAngleAddr, bottomServoAngle);
}

void setY(const int yVal) {
  topServo.write(topServoAngle);
  EEPROM.write(topAngleAddr, topServoAngle);
}

 
void autoFire() {
  int minAngleX=60;
  int maxAngleX=72;
  int scanAngle=38;
  int bestX=maxAngleX;
  int lowestDist=1000;
  
  rotateSlowlyTo(maxAngleX, scanAngle);
  delay(200);
  digitalWrite(laserPin, HIGH);

  //Find closest cup (X-direction)
  for (int i = maxAngleX; i >= minAngleX; i--) {
    rotateSlowlyTo(i, scanAngle);
    delay(500);
    int distance = 0;
    int strength = 0;
    getDistance(&distance,&strength);
    Serial.println("At xVal " + String(i) + " the distance is " + String(distance) + " strenth is " + String(strength));
    if (distance<lowestDist) {
      bestX=i;
      lowestDist=distance;
      Serial.println("new shortest " + String(bestX) + " with distance " + String(lowestDist));
    }
  }  
  Serial.println("Best x rot is " + String(bestX) + " with distance " + String(lowestDist));
  rotateSlowlyTo(bestX, scanAngle);
  delay(2000);

  //Find distance to closest cup
  for (int i = topServoHorizontalAngleMs-200; i < topServoHorizontalAngleMs; i++) {
    delay(10);
    topServo.writeMicroseconds(i);
    int distance = 0;
    int strength = 0;
    getDistance(&distance,&strength);
    Serial.println(String(i) + ";" + String(distance) + ";" + String(strength));
  }
  
  digitalWrite(laserPin, LOW);

  int angle=75;
  rotateSlowlyTo(bestX, angle);
  delay(200);
  fire();
}

void rotateSlowlyTo(const int xVal, const int yVal) {
  if (xVal < 0 || xVal > 180 || yVal < MIN_ANGLE_TOP_SERVO || yVal > MAX_ANGLE_TOP_SERVO) {
    Serial.println("Invalid rotation, " + String(xVal) + ", " + String(yVal));
    return;
  }

  while(bottomServoAngle!=xVal||topServoAngle!=yVal) {
    if (xVal<bottomServoAngle) {
        bottomServoAngle = bottomServoAngle - 1;
        setX(bottomServoAngle);
      } else if(xVal>bottomServoAngle) {
        bottomServoAngle = bottomServoAngle + 1;
        setX(bottomServoAngle);
      }

      if (yVal<topServoAngle) {
        topServoAngle = topServoAngle - 1;
        setY(topServoAngle);
      } else if(yVal>topServoAngle) {
        topServoAngle = topServoAngle + 1;
        setY(topServoAngle);
      }
      delay(20);
  }
}
 
void rotateDirectly(const int xVal, const int yVal) {
  if (xVal < 0 || xVal > 180 || yVal < MIN_ANGLE_TOP_SERVO || yVal > MAX_ANGLE_TOP_SERVO) {
    Serial.println("Invalid rotation, " + String(xVal) + ", " + String(yVal));
    return;
  }
  int delayTime = max(abs(bottomServoAngle-xVal), abs(topServoAngle-yVal)) * 20;
  bottomServoAngle = xVal;
  topServoAngle = yVal;

  bottomServo.write(bottomServoAngle);
  EEPROM.write(bottomAngleAddr, bottomServoAngle);
  topServo.write(topServoAngle);
  EEPROM.write(topAngleAddr, topServoAngle);
  
  delay(delayTime);
}
 
void rotate(const int value) {
  if (value == 'l') {
    bottomServoAngle = bottomServoAngle + 1;
    if (bottomServoAngle > 180) {
      bottomServoAngle = 180;
    }
    bottomServo.write(bottomServoAngle);
  }
  if (value == 'r') {
    bottomServoAngle = bottomServoAngle - 1;
    if (bottomServoAngle < 0) {
      bottomServoAngle = 0;
    }
    bottomServo.write(bottomServoAngle);
  }
  if (value == 'u') {
    topServoAngle= topServoAngle + 1;
    if (topServoAngle > MAX_ANGLE_TOP_SERVO) {
      topServoAngle = MAX_ANGLE_TOP_SERVO;
    }
    topServo.write(topServoAngle);
  }
  if (value == 'd') {
    topServoAngle= topServoAngle - 1;
    if (topServoAngle < MIN_ANGLE_TOP_SERVO) {
      topServoAngle = MIN_ANGLE_TOP_SERVO;
    }
    topServo.write(topServoAngle);
  }
  //Delay and write angles to EEPROM (local storage)
  delay(20);
  EEPROM.write(topAngleAddr, topServoAngle);
  EEPROM.write(bottomAngleAddr, bottomServoAngle);
  char buf[10];
  if (value == 'd' || value == 'u') {
    sprintf(buf, "y%d", topServoAngle);
  } else {
    sprintf(buf, "x%d", bottomServoAngle);
  }
 
  btSerial.println(buf);
}
 
void fire() {
  Serial.println("Fire!!");
  digitalWrite(fanPin, LOW);
  delay(500);
  digitalWrite(fanPin, HIGH);
}

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
                //Serial.print(t2);
                //Serial.print('\t');
                dist=t2;
    
                t1 = Serial.read(); //Byte5
                t2 = Serial.read(); //Byte6
    
                t2 <<= 8;
                t2 += t1;
                str=t2;
                //Serial.println(t2);
    
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
 
//Distance measure
void getTFminiData(int* distance, int* strength) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if(Serial.available()) {  
    Serial.println("Available");
    rx[i] = Serial.read();
    Serial.println("Read " + String(rx[i]));
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } else {
      i++;
    }
  }  
}
 
//HELPERS
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


 
struct Point {
    typedef Point P;
    double x, y;
    explicit Point(double _x, double _y) {
        x = _x;
        y = _y;
    }
    
    P operator+ (P p) const { return P(x + p.x, y + p.y); }
    P operator* (double d) const { return P(x * d, y * d); }
    P rotate(double a) const {
        return P(x * cos(a) - y * sin(a), x * sin(a) + y * cos(a));
    }
    
};

 
double calcDist(double angle) {
    double platformHeight = 14.2;
 
    double theta = M_PI * angle / 180;
    double platformToPipeLength = 12;
    double pipeLength = 33.5;
 
    Point l_vec = Point(platformToPipeLength, 0).rotate(theta);
    Point h_vec = (l_vec * (pipeLength / platformToPipeLength)).rotate(-M_PI / 2);
    Point final_point = l_vec + h_vec;
    
    return final_point.y + platformHeight;
}

