#include "Pid.h"
#include "unity.h"
#include <stdio.h>


void setUp() {
 // setup runs before each test
}


void tearDown() {
 // clean up runs before each test
}


// Tests intializing a Pid Stuct 
void testInit () {
  // Pid p = newPid(1,2,3,4,5,6,7,8,9,10,11,12); 
  // Pid p = newPid(1.1,2.2);
  // TEST_ASSERT_EQUAL(1, p.kp);
  // TEST_ASSERT_EQUAL(2, p.ki);
}

void testTsSetter() {}

void testSigmaSetter() {}

void testReset(){}

void testSetPointReset(){}


int main() {
  UNITY_BEGIN();
  RUN_TEST(testInit);


  return UNITY_END(); 
}
