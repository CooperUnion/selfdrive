#include "Pid.h"
#include "switch.h"
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
  Pid p = newPid(1,2,3,4,5,6,7,8,9,10,11,12); 
  // Pid p = newPid(1.1,2.2);
  TEST_ASSERT_EQUAL(1, p.kp);
  TEST_ASSERT_EQUAL(2, p.ki);
  TEST_ASSERT_EQUAL(3, p.kd);
  TEST_ASSERT_EQUAL(4, p.lower_lim);
  TEST_ASSERT_EQUAL(5, p.upper_lim);
  TEST_ASSERT_EQUAL(6, p._ts);
  TEST_ASSERT_EQUAL(7, p._sigma);
  TEST_ASSERT_EQUAL(8,p.beta);
  TEST_ASSERT_EQUAL(9,p.y0);
  TEST_ASSERT_EQUAL(10,p.err0);
  TEST_ASSERT_EQUAL(11,p.err_dot);
  TEST_ASSERT_EQUAL(12,p._int);
}

void testTsSetter() {
    tsSetter(p,1);
    TEST_ASSERT_EQUAL(1,p._ts);
    TEST_ASSERT_EQUAL((13/15),p.beta);
}

void testSigmaSetter() {
    sigmaSetter(p,7);
    TEST_ASSERT_EQUAL(7,p._sigma);
    TEST_ASSERT_EQUAL((8/20),p.beta);
}

void testReset(){
}

void testSetPointReset(){
    setpointReset(p,10,5);
    TEST_ASSERT_EQUAL(0,p._int);
    TEST_ASSERT_EQUAL(5,p,err0);
    TEST_ASSERT_EQUAL(0,p.err_dot);
}

void testPIDController_Update(){
    PIDController_Update(p,10,5);
   
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(testInit);

}