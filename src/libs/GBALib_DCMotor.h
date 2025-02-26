
/*
  GBALib_DCMotor.h - DC Motor Management Library
  Copyright (c) 2024 Graziano Blasilli.

  This library provides a simple interface to control DC motors using the L293D Motor Driver IC.
  It supports direction control, speed adjustment via PWM, and optional timed operation.
*/

#ifndef GBALib_DCMotor
#define GBALib_DCMotor

#include <Arduino.h>

/**
 * DCMotor class for managing a DC motor using the L293D Motor Driver IC.
 * Provides methods for controlling speed, direction, and runtime.
 */
class DCMotor {
  private:
    uint8_t pinA;  // Motor input pin A on the L293D
    uint8_t pinB;  // Motor input pin B on the L293D
    uint8_t pinS;  // Motor enable pin (PWM) on the L293D
  
  public:
    /**
     * Constructor to initialize the DCMotor object and set pin modes.
     * 
     * @param pinA Motor input pin A.
     * @param pinB Motor input pin B.
     * @param pinS Motor enable pin (PWM).
     */
    DCMotor(uint8_t pinA, uint8_t pinB, uint8_t pinS);

    /**
     * Turn the motor on with a specified speed.
     * 
     * @param speed Speed of the motor (-255 to 255). Negative values reverse direction.
     */
    void on(int speed);

    /**
     * Turn the motor on with a specified speed, for a given time.
     * 
     * @param speed Speed of the motor (-255 to 255).
     * @param millisec Duration to run the motor in milliseconds.
     */
    void on(int speed, unsigned long millisec);

    /**
     * Turn the motor off.
     */
    void off();
};

#endif