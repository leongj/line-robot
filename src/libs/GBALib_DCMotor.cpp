
/*
  GBALib_DCMotor.cpp - DC Motor Management Library
  Copyright (c) 2024 Graziano Blasilli.

  This library provides a simple interface to control DC motors using the L293D Motor Driver IC.
  It supports direction control, speed adjustment via PWM, and optional timed operation.
*/

#include "GBALib_DCMotor.h"

// Constructor to initialize the DCMotor object and set pin modes
DCMotor::DCMotor(uint8_t pinA, uint8_t pinB, uint8_t pinS)
    : pinA(pinA), pinB(pinB), pinS(pinS) {
  // Set the motor control pins as output
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinS, OUTPUT);
}

// Turn the motor on with a specified speed
void DCMotor::on(int speed) {
  this->off(); // Stop the motor before setting a new state

  // Clamp the speed value between -255 and 255 for valid PWM
  speed = constrain(speed, -255, 255);

  // Set motor direction based on the sign of the speed
  digitalWrite(this->pinA, speed >= 0 ? HIGH : LOW);
  digitalWrite(this->pinB, speed >= 0 ? LOW : HIGH);

  // Set motor speed using PWM (absolute value to handle negative speeds)
  analogWrite(this->pinS, abs(speed));
}

// Turn the motor on with a specified speed, for a given time
void DCMotor::on(int speed, unsigned long millisec) {
  this->on(speed);     // Turn the motor on at the specified speed
  delay(millisec);     // Wait for the specified duration
  this->off();         // Turn the motor off after the delay
}

// Turn the motor off
void DCMotor::off() {
  // Set both direction pins to LOW and stop PWM signal
  digitalWrite(this->pinA, LOW);
  digitalWrite(this->pinB, LOW);
  analogWrite(this->pinS, 0);
}