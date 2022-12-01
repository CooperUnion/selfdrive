
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
void testInit (void) {
  Pid p = newPid(1,2,3,4,5,6,7); 
  // Pid p = newPid(1.1,2.2);
  TEST_ASSERT_EQUAL(1, p.kp);
  TEST_ASSERT_EQUAL(2, p.ki);
  TEST_ASSERT_EQUAL(3, p.kd);
  TEST_ASSERT_EQUAL(4, p.lower_lim);
  TEST_ASSERT_EQUAL(5, p.upper_lim);
  TEST_ASSERT_EQUAL(6, p._ts);
  TEST_ASSERT_EQUAL(7, p._sigma);
  TEST_ASSERT_EQUAL(1,p.beta);
  TEST_ASSERT_EQUAL(0.0,p.y0);
  TEST_ASSERT_EQUAL(0.0,p.err0);
  TEST_ASSERT_EQUAL(0.0,p.err_dot);
  TEST_ASSERT_EQUAL(0.0,p._int);
}

void testTsSetter() {
  Pid p = newPid(1,2,3,4,5,6,7); 
  tsSetter(&p,1);
  TEST_ASSERT_EQUAL(1, p._ts);
  TEST_ASSERT_EQUAL((13/15), p.beta);
    
}

void testSigmaSetter() {
  Pid p=startup(1,2,3,4,5,6,7);
  sigmaSetter(&p,7);
  TEST_ASSERT_EQUAL(7,p._sigma);
  TEST_ASSERT_EQUAL((8/20),p.beta);
  
}

void testReset(){
}

void testSetPointReset(){
    Pid p=startup(1,2,3,4,5,6,7);
    setpointReset(&p,10,5);
    TEST_ASSERT_EQUAL(0,p._int);
    TEST_ASSERT_EQUAL(5,p.err0);
    TEST_ASSERT_EQUAL(0,p.err_dot);
}

void testPIDController_Update(){
    //PIDController_Update(p,10,5);
   
}
void testacc_to_pedal(){
   TEST_ASSERT_EQUAL(15.4,acc_to_pedal(1.0));
   
}
void testbrake_to_pedal(){
   TEST_ASSERT_EQUAL(-49.13,brake_to_pedal(1.0));
   
}
void testenc_to_velocity(){
   TEST_ASSERT_EQUAL(0.474789,enc_to_velocity(1.0,1000.0));
}
void testbrake_or_throttle(){
//1st case
  //vactual==-.4, accdes=v_des=1 
  //BreakOrThrottle bT= newbT(0.0,15.4);
  //BreakOrThrottle bT_act=brake_or_throttle(-.4,1,1);
//2nd case
  //vactual==-.4, accdes=v_des=-1 
  //BreakOrThrottle bT= newbT(49.13,0.0);
  //BreakOrThrottle bT_act=brake_or_throttle(-.4,-1,-1);
//3rd case
  //vactual==-.6, accdes=v_des=1 
  //BreakOrThrottle bT= newbT(50,0.0);
  //BreakOrThrottle bT_act=brake_or_throttle(-.6,1,1);
//4th case
  //vactual==1, accdes=v_des=1 
  //BreakOrThrottle bT= newbT(0.0,15.4);
  //BreakOrThrottle bT_act=brake_or_throttle(1,1,1);
//5th case
  //vactual==1, accdes=v_des=-1 
  BreakOrThrottle bT= newbT(49.13,0.0);
  BreakOrThrottle bT_act=brake_or_throttle(1,-1,-1);
//6th case-can't really test
  //vactual==not a numner/error, accdes=v_des=1 --doesnt really matter
  TEST_ASSERT_EQUAL(bT.throttle_percentage,bT_act.throttle_percentage);
  TEST_ASSERT_EQUAL(bT.brake_percentage,bT_act.brake_percentage);
   
}

int main() {
  //testPIDController_Update,,,);
  UNITY_BEGIN();
  RUN_TEST(testInit);
  RUN_TEST(testTsSetter);
  RUN_TEST(testSigmaSetter);
  RUN_TEST(testReset);
  RUN_TEST(testSetPointReset);
  RUN_TEST(testacc_to_pedal);
  RUN_TEST(testbrake_to_pedal);
  RUN_TEST(testenc_to_velocity);
  RUN_TEST(testbrake_or_throttle);
  return UNITY_END(); 

}