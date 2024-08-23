#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String msg;

String state;

// Pin definitions based on your provided configuration
int left_IN1 = 9;   // IN1
int left_IN2 = 8;   // IN2
int right_IN3 = 5;  // IN3
int right_IN4 = 4;  // IN4
int left_ENA = 10;  // ENA
int right_ENB = 3;  // ENB

void callback(const std_msgs::String& control) {
  state = control.data;

  if (state == "Left") {
    Serial.println("Left");
    digitalWrite(left_IN1, LOW);
    digitalWrite(left_IN2, HIGH);
    digitalWrite(right_IN3, HIGH);
    digitalWrite(right_IN4, LOW);
    analogWrite(left_ENA, 0);  // Slow down left motor
    analogWrite(right_ENB, 255);  // Speed up right motor
    delay(50);
  } else if (state == "Right") {
    Serial.println("Right");
    digitalWrite(left_IN1, HIGH);
    digitalWrite(left_IN2, LOW);
    digitalWrite(right_IN3, LOW);
    digitalWrite(right_IN4, HIGH);
    analogWrite(left_ENA, 255);  // Speed up left motor
    analogWrite(right_ENB, 0);  // Slow down right motor
    delay(50);
  } else if (state == "Go") {
    Serial.println("GO");
    digitalWrite(left_IN1, HIGH);
    digitalWrite(left_IN2, LOW);
    digitalWrite(right_IN3, HIGH);
    digitalWrite(right_IN4, LOW);
    analogWrite(left_ENA, 255);  // Move forward with moderate speed
    analogWrite(right_ENB, 255);
    delay(100);
  } else if (state == "Stop") {
    Serial.println("Stop");
    digitalWrite(left_IN1, LOW);
    digitalWrite(left_IN2, LOW);
    digitalWrite(right_IN3, LOW);
    digitalWrite(right_IN4, LOW);
    analogWrite(left_ENA, 0);  // Stop left motor
    analogWrite(right_ENB, 0);  // Stop right motor
    delay(100);
  }  
}

ros::Subscriber<std_msgs::String> sub("/control", callback);  // Subscribe to the cv_detection topic

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(left_IN1, OUTPUT);
  pinMode(left_IN2, OUTPUT);
  pinMode(right_IN3, OUTPUT);
  pinMode(right_IN4, OUTPUT);
  pinMode(left_ENA, OUTPUT);
  pinMode(right_ENB, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
