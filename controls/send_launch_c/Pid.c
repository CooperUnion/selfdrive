#include "./Pid.h"

Pid newPid(double a, double b) {
  Pid p; 
  p.a = a; 
  p.b = b; 
  return p;
}
