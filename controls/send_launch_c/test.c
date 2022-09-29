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
  Pid p = newPid(1.1,2.2);
  TEST_ASSERT_EQUAL(1.1, p.a);
  TEST_ASSERT_EQUAL(2.2, p.b);
}


int main() {
  UNITY_BEGIN();
  RUN_TEST(testInit);


  return UNITY_END(); 
}
