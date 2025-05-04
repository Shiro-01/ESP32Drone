// PIDController.h

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float last_derivative;
    float T;       // Cutoff period for the low pass filter

    PIDController(float p, float i, float d, float t) : Kp(p), Ki(i), Kd(d), T(t), integral(0), last_error(0), last_derivative(0) {}

    float compute(float target, float current, float dt, float maxOutput, float minOutput);
    void reset();
    void setTunings(float p, float i, float d, float t);

};

#endif // PIDCONTROLLER_H
