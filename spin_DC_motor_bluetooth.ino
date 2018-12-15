#include <SoftwareSerial.h>

const int rxpin = 0;
const int txpin = 1;
char k = 'L';
int motorPin = 10;
SoftwareSerial bluetooth(rxpin, txpin);
String s = "";

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7
int pwmOutputA = 0;
int pwmOutputB = 0;
bool motor1 = false;
bool motor2 = false;
bool motor3 = false;
bool motor4 = false;
int command1;
int command2;

void setup()
{
  bluetooth.begin(9600);
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  command1 = 0;
  command2 = 0;
}

void loop()
{
  if (bluetooth.available()) {
    k = bluetooth.read();
    s += k;
    //    Serial.print("bluetooth received: ");
    Serial.println(s);
    Serial.println("new command");
    delay(100);
//    bluetooth.write("new command");
    command1 = 0;
    command2 = 0;
  } else {
    Serial.println(s);
    if (s.length() > 0) {
      int commaIndex = s.indexOf(',');
      String s1 = s.substring(0, commaIndex);
      String s2 = s.substring(commaIndex + 1);
      command1 = s1.toInt();
      command2 = s2.toInt();
      s = "";
      motor1 = false;
      motor2 = false;
      motor3 = false;
      motor4 = false;
    }
    Serial.print(command1);
    Serial.println(command2);
    if ((command1 > 0 && command2 > 0) || motor1) {
      move_forward(command1, command2);
      bluetooth.write("motor 1 on");
      motor1 = true;
    } else if ((command1 < 0 && command2 < 0) || motor2)  {
      move_backward(command1, command2);
      bluetooth.write("motor 2 on");
      motor2 = true;
    } else if ((command1 > 0 && command2 <= 0) || motor3)  {
      swivel_right(command1, command2);
      bluetooth.write("motor 3 on");
      motor3 = true;
    } else if ((command1 <= 0 && command2 > 0) || motor4)  {
      swivel_left(command1, command2);
      bluetooth.write("motor 4 on");
      motor4 = true;
    } else {
      stop_motors();
      motor1 = false;
      motor2 = false;
      motor3 = false;
      motor4 = false;
    }
  }
  delay(10);
}

void move_forward(int l, int r) {
  digitalWrite(in1, HIGH); // might have to change these values based on how the motor is wired
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  pwmOutputA = abs(l);
  pwmOutputB = abs(r);
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(20);
  //  Serial.println("moving forward");
}
void swivel_left(int l, int r) {
  // eventually swivel will be a function of degrees
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); //rn assuming 3,4 is the combo that is working haha
  pwmOutputA = abs(l);
  pwmOutputB = abs(r);
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(20);
}
void swivel_right(int l, int r) {
  // eventually swivel will be a function of degrees
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); //rn assuming 3,4 is the combo that is working haha
  pwmOutputA = abs(l);
  pwmOutputB = abs(r);
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(20);
}
void stop_motors() {
  pwmOutputA = 0;
  pwmOutputB = 0;
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(20);
}
void one_turn() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); //rn assuming 3,4 is the combo that is working haha
  pwmOutputA = 200;
  pwmOutputB = 200;
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(4333);
  pwmOutputA = 0;
  pwmOutputB = 0;
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(2000);
}
void move_backward(int l, int r) {
  digitalWrite(in1, LOW); // might have to change these values based on how the motor is wired
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  pwmOutputA = abs(l);
  pwmOutputB = abs(r);
  analogWrite(enA, pwmOutputA);
  analogWrite(enB, pwmOutputB);
  delay(20);
  //  Serial.println("moving backwards");
}
