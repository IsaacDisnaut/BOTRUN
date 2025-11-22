#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TRIG_LEFT 2
#define ECHO_LEFT 4

#define TRIG_FRONT 11
#define ECHO_FRONT 12

#define TRIG_RIGHT 7
#define ECHO_RIGHT 6

SoftwareSerial mySerial(A0, A1); // RX, TX
bool isBusy = false;
const int enPinX = 9;
const int in1X = 3;
// const int in2X = 4;

const int enPinY = 10;
const int in1Y = 5;
// const int in2Y = 6;
double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
int facing = 0;
int motorSpeed = 75; // 0–255 PWM speed
int motorSpeed2 = 65;
uint16_t BNO055_SAMPLERATE_DELAY_MS =100;
int distancetowall = 18;
int phase =1;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
int xx,yy,facingdum,tarx,tary,prevxx=0,prevyy=0;
String go;
int timedelay = 1000;
int delayforward = 2000;
void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(in1X, OUTPUT);
  pinMode(enPinX, OUTPUT);

  pinMode(in1Y, OUTPUT);
  pinMode(enPinY, OUTPUT);

  // ตั้งค่าค่าเริ่มต้น
  digitalWrite(in1X, LOW);
  digitalWrite(in1Y, LOW);

  // ตั้งค่าความเร็วเริ่มต้น (0–255)
  analogWrite(enPinX, 0);
  analogWrite(enPinY, 0);
  Serial.println("✅ Ready for DC motor commands");

  /* Initialise the sensor */
 if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);



}

String command;
int duration=200;

  sensors_event_t orientationData;
void loop() {
  float leftDist, frontDist, rightDist;
  readAllUltrasonic(leftDist, frontDist, rightDist);

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  printEvent(&orientationData);
  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");
if (mySerial.available()) {

    String data = mySerial.readStringUntil('\n');
data.trim();

// แยกด้วย '/'
int p1 = data.indexOf('/');
int p2 = data.indexOf('/', p1 + 1);
int p3 = data.indexOf('/', p2 + 1);

String command = data.substring(0, p1);        // "xx"
String duration = data.substring(p1 + 1, p2);   // "1"
String command2 = data.substring(p2 + 1, p3);   // "yy"
String duration2= data.substring(p3 + 1);       // "7"

if (command == "xx") {
    xx = duration.toInt();
}
if (command2 == "yy") {
    yy = duration2.toInt();
}
  if(command == "phase"){
    phase = duration.toInt();
  }
Serial.print("xx = "); Serial.println(xx);
Serial.print("yy = "); Serial.println(yy);

}
      if(phase == 1){
Serial.print("phase1");
      //check_exit();


      if (rightDist >= distancetowall) {
        TurnRight();
        delay(100);
        ForwardMap();

      } 
      else if (frontDist >= distancetowall) {
        mySerial.println("forwardmap");
        ForwardMap();
        
        
      } 
      else if (leftDist >= distancetowall) {
        TurnLeft();
        delay(100);
        ForwardMap();
      } 
      else {
        TurnAround();
        delay(100);
        ForwardMap();

      }
    }
    else if(phase == 2)
    {
      Serial.print("phase2");
    String data = mySerial.readStringUntil('\n');
    data.trim();
    int slashIndex = data.indexOf('/');
    if (slashIndex != -1) {
       command = data.substring(0, slashIndex);
      duration = data.substring(slashIndex + 1).toInt();

      Serial.print("Command: "); Serial.print(command);
      Serial.print(" | Duration: "); Serial.println(duration);
    }
     if(command=="reach")
      {
        if(facing == facingdum)
        {
          if(go == "left")
          {
            
            TurnLeft();
            ForwardMap();
        }
        if(go == "right")
        {
            TurnRight();
            ForwardMap();
        }
        if(go == "forward")
        {
          ForwardMap();
          mySerial.println("forwardmap");
        }
      }
      else {
        while(facing != facingdum)
        {
          TurnRight();
          delay(100);
        }
        if(go == "left")
          {
            TurnLeft();
            ForwardMap();
        }
        if(go == "right")
        {
            TurnRight();
            ForwardMap();
        }
        if(go == "forward")
        {
          ForwardMap();
          mySerial.println("forwardmap");
        }
      }
      }

      if(rightDist >= distancetowall || leftDist >= distancetowall)
      {
        delay(500);
        StopMotors();
        mySerial.println("ask");
          if (command == "right") {
        TurnRight();
        delay(100);
        ForwardMap();
        Serial.println("Right");

      } 
      else if (command == "forward") {
        ForwardMap();
        mySerial.println("forwardmap");
        
      } 
      else if (command == "left") {
        TurnLeft();
        delay(100);
        ForwardMap();
      } 
      }
      else{
        ForwardMap();
        mySerial.println("forwardmap");
      }
     
    }
      
  if (command == "status") {
  if (isBusy) mySerial.println("busy");
  else mySerial.println("ready");
}
      command = "";
  prevxx=xx;
  prevyy=yy;
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  

void Forward(int duration) {
  digitalWrite(in1X, HIGH);
  // digitalWrite(in2X, LOW);
  digitalWrite(in1Y, HIGH);
  // digitalWrite(in2Y, LOW);

  analogWrite(enPinX, motorSpeed);
  analogWrite(enPinY, motorSpeed);

  delay(duration);

  StopMotors();
}

void TurnLeft() {
  // left motor reverse, right motor forward
    
  isBusy = true;
  digitalWrite(in1X, LOW);
  // digitalWrite(in2X, HIGH);
  digitalWrite(in1Y, HIGH);
  // digitalWrite(in2Y, LOW);

  analogWrite(enPinX, motorSpeed);
  analogWrite(enPinY, motorSpeed);

  facing = (facing - 90) % 360;
while (true) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    printEvent(&orientationData);
    float yaw = x;
    if (yaw < 0) yaw += 360;  // normalize 0–360

    float error = angleError(facing, yaw); // -180..+180

    Serial.print("left end now yaw= ");
    Serial.println(yaw);

    if (abs(error) <= 0.5) {  // ตอนนี้เปรียบเทียบถูกต้อง
        StopMotors();
        break;
    }
}
  StopMotors();
  Serial.print("Rotate cumplete");
  delay(500);
  mySerial.println("left");

  isBusy = false;
}

void TurnRight() {
  // right motor reverse, left motor forward
  isBusy = true;
  digitalWrite(in1X, HIGH);
  // digitalWrite(in2X, LOW);
  digitalWrite(in1Y, LOW);
  // digitalWrite(in2Y, HIGH);

  analogWrite(enPinX, motorSpeed);
  analogWrite(enPinY, motorSpeed);

  facing = (facing + 90) % 360;
 while (true) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    printEvent(&orientationData);
    float yaw = x;
    if (yaw < 0) yaw += 360;  // normalize 0–360

    float error = angleError(facing, yaw); // -180..+180

    Serial.print("right end now yaw= ");
    Serial.println(yaw);

    if (abs(error) <= 0.5) {  // ตอนนี้เปรียบเทียบถูกต้อง
        StopMotors();
        break;
    }
}

  Serial.print("Rotate cumplete");
    mySerial.println("right");
  delay(500);

  
  isBusy = false;
  Serial.println("Right");

}

void TurnAround() {
  // right motor reverse, left motor forward
  isBusy = true;
  digitalWrite(in1X, HIGH);
  // digitalWrite(in2X, LOW);
  digitalWrite(in1Y, LOW);
  // digitalWrite(in2Y, HIGH);

  analogWrite(enPinX, motorSpeed);
  analogWrite(enPinY, motorSpeed);

  facing = (facing + 180) % 360;

while (true) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    printEvent(&orientationData);
    float yaw = x;
    if (yaw < 0) yaw += 360;  // normalize 0–360

    float error = angleError(facing, yaw); // -180..+180

    Serial.print("left end now yaw= ");
    Serial.println(yaw);

    if (abs(error) <= 0.5) {  // ตอนนี้เปรียบเทียบถูกต้อง
        StopMotors();
        break;
    }
}

delay(500);
mySerial.println("around");
Serial.print("Rotate cumplete");
  
  isBusy = false;
}

void StopMotors() {
  digitalWrite(in1X, LOW);
  // digitalWrite(in2X, LOW);
  digitalWrite(in1Y, LOW);
  // digitalWrite(in2Y, LOW);
  analogWrite(enPinX, 0);
  analogWrite(enPinY, 0);
  delay(100);
}



float angleError(float target, float current) {
  float diff = fmod((target - current + 540.0), 360.0) - 180.0;
  Serial.print("diff=");Serial.println(diff);
  return diff;
}

// ---------------------------------------------------------

float kp = 10.0;
float ki = 0.1;
float kd = 0.1;

float lastError = 0;
float integral = 0;
// ---------------------------------------------------------
void ForwardMap() {
  isBusy = true;
  String data1 = "";
  String command1 = "";

  Serial.println("✅ Forward mapping mode started");

  unsigned long startTime = millis();
  int duration1;

  // ตั้งให้รถวิ่งไปข้างหน้า
  digitalWrite(in1X, HIGH);
  digitalWrite(in1Y, HIGH);

  analogWrite(enPinX, motorSpeed);
  analogWrite(enPinY, motorSpeed2);

  while (true) {

    // ------------------- อ่านคำสั่งจาก Python -------------------
    if (mySerial.available()) {
        String data1 ="";
        data1.trim();
         data1 = mySerial.readStringUntil('\n');


        int p1 = data1.indexOf('/');
        int p2 = data1.indexOf('/', p1 + 1);
        int p3 = data1.indexOf('/', p2 + 1);

        String cmd1 = data1.substring(0, p1);
        String val1 = data1.substring(p1 + 1, p2);
        String cmd2 = data1.substring(p2 + 1, p3);
        String val2 = data1.substring(p3 + 1);

        if (cmd1 == "xx") xx = val1.toInt();
        if (cmd2 == "yy") yy = val2.toInt();

        if (data1 == "stop") {
            Serial.println("🛑 Stop command received!");
            delay(300);
            StopMotors();
            delay(500);
            mySerial.println("stop");
            break;
        }
    }

    // ============================
    // 1) อ่าน IMU → PID ทิศทาง
    // ============================
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    printEvent(&orientationData);

    float error = angleError(facing, x);  // ต้องการให้หันไปตาม facing
    integral += error;
    float derivative = error - lastError;
    lastError = error;

    float pid_correction = kp * error + ki * integral + kd * derivative;


    // ============================
    // 2) อ่าน Ultrasonic → Wall Avoidance
    // ============================
    float leftDist, frontDist, rightDist;
    readAllUltrasonic(leftDist, frontDist, rightDist);

    float wallThreshold = 11.0;  // ระยะที่เริ่มเลี่ยง (cm)
    float Wgain = 3;           // ความแรงของการเลี่ยงกำแพง
    float wall_avoid = 0;

    // ถ้ากำแพงซ้ายใกล้ → เบี่ยงขวา
    if (leftDist > 0 && leftDist < wallThreshold) {
        wall_avoid += Wgain * (wallThreshold - leftDist);
    }

    // ถ้ากำแพงขวาใกล้ → เบี่ยงซ้าย
    if (rightDist > 0 && rightDist < wallThreshold) {
        wall_avoid -= Wgain * (wallThreshold - rightDist);
    }

    // กำแพงด้านหน้าใกล้ → ลดความเร็ว
   


    // ============================
    // 3) รวม PID + Wall Avoidance
    // ============================
    float correction_total = pid_correction + wall_avoid;

    int leftSpeed  = constrain(motorSpeed  + correction_total, 0, 100);
    int rightSpeed = constrain(motorSpeed2 - correction_total, 0, 100);


    // ============================
    // 4) สั่งมอเตอร์
    // ============================
    analogWrite(enPinX, leftSpeed);
    analogWrite(enPinY, rightSpeed);


    // ---------------- Debug Monitor ----------------
    Serial.print("Yaw: ");
    Serial.print(x);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | PID=");
    Serial.print(pid_correction);
    Serial.print(" | Wall=");
    Serial.print(wall_avoid);
    Serial.print(" | L=");
    Serial.print(leftSpeed);
    Serial.print(" | R=");
    Serial.println(rightSpeed);
}


  isBusy = false;
  delay(300);
}



void check_exit() {
  float leftDist, frontDist, rightDist;
  readAllUltrasonic(leftDist, frontDist, rightDist);

  if (yy == 0) {
    Serial.println("Now y=0");
    if (facing == 0) {
      if (leftDist >= distancetowall) { // left
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "left";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forward");
        mySerial.println("forwardmap");

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();
   
      }
    }

    if (facing == -90 || facing == 270) { // forward
      if (frontDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "forward";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (abs(facing) == 180) { // right
      if (rightDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "right";
        Serial.println(state);
        mySerial.print(state);
      }
      if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forward");
        mySerial.println("forwardmap");

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }
  }

  else if (yy == 9) {
    if (facing == 0) {
      if (rightDist >= distancetowall) { // right
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "forward";
        Serial.println(state);
        mySerial.print(state);
      }
      if (frontDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();
      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (abs(facing) == 180) {
      if (leftDist >= distancetowall) { // left
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "left";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forward");
        mySerial.println("forwardmap");

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (facing == 90 || facing == -270) { // forward
      if (frontDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "right";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();
      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();
      }
    }
  }

  else if (xx == 0) {
    if (facing == 90 || facing == -270) { // right
      if (rightDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "right";
        Serial.println(state);
        mySerial.print(state);
      }
      if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forward");
        mySerial.println("forwardmap");

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("LEft");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (abs(facing) == 180) { // forward
      if (frontDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "forward";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("LEft");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (facing == -90 || facing == 270) {
      if (leftDist >= distancetowall) { // left
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "left";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forward");
        mySerial.println("forwardmap");
   
      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }
  }

  else if (xx == 9) {
    if (facing == 0) {
      if (frontDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "forward";
        Serial.println(state);
        mySerial.print(state);
      }
      if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (facing == 90 || facing == -270) {
      if (leftDist >= distancetowall) { // left
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "left";
        Serial.println(state);
        mySerial.print(state);
      } else if (rightDist >= distancetowall) {
        TurnRight();
        Serial.println("Right");
        ForwardMap();

      }
      if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forwardmap");
        mySerial.println("forwardmap");

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();

      }
    }

    if (facing == -90 || facing == 270) {
      if (rightDist >= distancetowall) {
        String state = "final";
        facingdum = facing;
        tarx = xx;
        tary = yy;
        go = "right";
        Serial.println(state);
        mySerial.print(state);
      }
      if (frontDist >= distancetowall) {
        ForwardMap();
        Serial.println("Forwardmap");
        mySerial.println("forwardmap");

      } else if (leftDist >= distancetowall) {
        TurnLeft();
        Serial.println("Left");
        ForwardMap();

      } else {
        TurnAround();
        Serial.println("Around");
        ForwardMap();
      }
    }
  }
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // จำกัดเวลา 25ms
  float distance = duration * 0.0343 / 2; // หน่วย: เซนติเมตร
  if (distance == 0 || distance > 50) distance = 50; // จำกัดค่า
  return distance;
}

void readAllUltrasonic(float &left, float &front, float &right) {
  left  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  delay(80);  // พักให้เสียงสะท้อนหมดจากตัวแรก
  front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(80);  // พักให้เสียงสะท้อนหมดจากตัวแรก
  right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  delay(80);  // เว้นช่วงรวม ~240 ms
}

void printEvent(sensors_event_t* event) {
 
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
}
