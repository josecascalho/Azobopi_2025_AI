#include "Arduino.h"
#include "PID_simple.h"

PID::PID(double *setpoint, double *encread, double *output, double kp, double ki, double kd)
  : _setpoint(setpoint),
    _encread(encread),
    _output(output),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    dispP(0),
    dispI(0),
    dispD(0),
    last_proportional(0),
    accumulated_integral(0)
{}

void PID::Reset() {
    accumulated_integral = 0;
    last_proportional = 0;
}

bool PID::Compute() {
  double error, proportional, integral, derivative;

  error             = *_setpoint - *_encread;
  proportional      = error;
  accumulated_integral += proportional;
  derivative        = proportional - last_proportional;
  last_proportional = proportional;
  *_output          = (proportional * _kp) + (accumulated_integral * _ki) +
                      (derivative * _kd);
  dispP = proportional;
  dispI = accumulated_integral;
  dispD = derivative;

  if (*_output) {
    return true;
  }
  else return false;
}

void PID::SetP(double p) {
  _kp = p;
}

void PID::SetI(double i) {
  _ki = i;
}

void PID::SetD(double d) {
  _kd = d;
}

double PID::GetP() {
  return dispP;
}

double PID::GetI() {
  return dispI;
}

double PID::GetD() {
  return dispD;
}
