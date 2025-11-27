/*
 * MECOTRON TUTORIAL - SOLUTION
 *
 * This is a possible solution for the PID controller that
 * is suggested in the tutorial session (part 2.7).
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  return true;
}

void Robot::control() {

  // Read the motor speed
  float vA = getSpeedMotorA();  
  float vB = getSpeedMotorB();  

  if(controlEnabled()) {
    LED1(ON);   // turn on LED 1
    LED2(OFF);  // turn off LED 2

    float r = 12;     
    float eB = r-vB;       
    float eA = r-vA;                
    float uB = n0d0*eB + n1d0*errorB_prev - d1d0*controlB_prev;             
    float uA = n0d0*eA + n1d0*errorA_prev - d1d0*controlA_prev; 
    errorA_prev = eA; errorB_prev = eB; controlA_prev = uA; controlB_prev = uB;    // append the new values

    setVoltageMotorA(uA);
    setVoltageMotorB(uB);

   
    writeValue(2, r);     
    writeValue(3, eA);    
    writeValue(4, eB);    
    writeValue(5, uA);    
    writeValue(6, uB);    

  } else {

    LED1(OFF); // turn off LED 2
    LED2(ON);  // turn on LED 2
    setVoltageMotorA(0.0);  // don't move motor A
    setVoltageMotorB(0.0);  // don't move motor B
  }

  writeValue(7, vA);
  writeValue(8, vB);

}

void Robot::resetController(){
  // Set all errors and control signals in the memory back to 0

    errorA_previous = 0.0;
    errorB_previous = 0.0;
    controlA_previous = 0.0;
    controlB_previous = 0.0;
  
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

void Robot::button0callback() {
  if(toggleButton(0)) {                          // Switches the state of button 0 and checks if the new state is true
    resetController();
    message("Controller reset and enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Robot disabled.");
  }
}

void Robot::button1callback() {
  toggleButton(1);
  init();                         // Reset the MECOtron and reinitialize the Robot object
  message("Reset.");
}
