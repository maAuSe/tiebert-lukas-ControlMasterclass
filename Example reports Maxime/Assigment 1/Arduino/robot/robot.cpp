/*
 * MECOTRON TUTORIAL
 *
 * This is a template to get you started in the course of the tutorial on the
 * control theory platforms, a.k.a. the MECOtrons.s
 * The tasks of the tutorial session will guide you through this template and
 * ask you to make use of the platform's capabilities in a step-by-step fashion.
 *
 * Every function in this template comes with an opening comment that describes
 * its purpose and functionality. Please also pay attention to the remarks that
 * are made in comment blocks.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron
  
  // Initializing the robot's specific variables
  for(int k=0; k<2; k++){
    x[k] = 0.0;   // Set all components of the vector (float array) x to 0 as initialization
  }
  timer = 0;
  return true;
}

void Robot::control() {

  // Compute update of motor voltages if controller is enabled (triggered by
  // pushing 'Button 0' in QRoboticsCenter)
  if(controlEnabled()) {
    // Fill your control law here to conditionally update the motor voltage...
    LED1(ON);
    LED2(OFF);
    timer = timer + 1;
  
    if (timer < 400) {
      setVoltageMotorA(0.0);
      setVoltageMotorB(0.0);
    } 
    else if ((timer > 400) && (timer < 800)) {
      setVoltageMotorA(8.0);
      setVoltageMotorB(8.0);   
    }
    else if ((timer > 800) && (timer < 1200)) {
      setVoltageMotorA(0.0);
      setVoltageMotorB(0.0);   
    }
    else if ((timer > 1200) && (timer < 1600)) {
      setVoltageMotorA(-8.0);
      setVoltageMotorB(-8.0);   
    }
    else if ((timer > 1600) && (timer < 2000)) {
      setVoltageMotorA(0.0);
      setVoltageMotorB(0.0);   
    }
    else if ((timer > 2000) && (timer < 2400)) {
      setVoltageMotorA(8.0);
      setVoltageMotorB(8.0);   
    }
    else if ((timer > 2400) && (timer < 2800)) {
      setVoltageMotorA(0.0);
      setVoltageMotorB(0.0);   
    }
    else if ((timer > 2800) && (timer < 3200)) {
      setVoltageMotorA(-8.0);
      setVoltageMotorB(-8.0);   
    }
    else if (timer > 3200) {
      setVoltageMotorA(0.0);
      setVoltageMotorB(0.0);  
    }
  }
  else {
    // If the controller is disabled, you might want to do something else...
    LED1(OFF);
    LED2(ON);
    setVoltageMotorA(0.0); // Apply 0.0 volts to motor A if the control is disabled
    setVoltageMotorB(0.0); // Apply 0.0 volts to motor B if the control is disabled
    timer = 0;
  }
  
  writeValue(0,timer);   
  writeValue(1,getSpeedMotorA()); 
  writeValue(2,getSpeedMotorB()); 
  writeValue(3,getVoltageMotorA());
  writeValue(4,getVoltageMotorB());

}


bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    message("Robot enabled.");    // Display a message in the status bar of QRoboticsCenter
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
