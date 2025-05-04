
// PIDController.cpp
#include "PIDController.h"
#include <Arduino.h>

float PIDController::compute(float target, float current, float dt, float maxOutput, float minOutput) {
    float error = target - current;

    // Low-pass filter for derivative
    float derivative = (error - last_error) / dt;
    float alpha = T / (T + dt);  // T is your filter time constant
    float filtered_derivative = alpha * last_derivative + (1 - alpha) * derivative;

    // Tentative integral calculation
    float tentative_integral = integral + error * dt;

    // Compute full output using tentative integral
    float output = (Kp * error) + (Ki * tentative_integral) + (Kd * filtered_derivative);

    // Anti-windup logic: Only apply the tentative integral if:
    // 1. Output is within limits
    // 2. OR the error is acting to reduce the integral (opposite signs)
    if ((output >= minOutput && output <= maxOutput) || (tentative_integral  * error < 0)) {
        integral = tentative_integral;
    }

    // Serial.print(" integral: "); Serial.println(integral); // for debuging only
    // Update state for next cycle
    last_error = error;
    last_derivative = filtered_derivative;

    // Recalculate output using updated integral
    output = (Kp * error) + (Ki * integral) + (Kd * filtered_derivative);
    output = constrain(output, minOutput, maxOutput);

    return output;
}

void PIDController::reset() {
    integral = 0;
    last_error = 0;
    last_derivative = 0;
}

void PIDController::setTunings(float p, float i, float d, float t) {
    Kp = p;
    Ki = i;
    Kd = d;
    T = t;
}

