/*
 * LINEAR KALMAN FILTER TEMPLATE
 *
 * This is a template to get you started with the implementation of the Kalman filter
 * on your own cart.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  desiredVelocityCart(0) = 0.0;
  return true;
}

void Robot::control() {

  float volt_A = 0.0;
  float volt_B = 0.0;
  float desiredVelocityMotorA = 0.0;
  float desiredVelocityMotorB = 0.0;
  Matrix<1> desiredVelocityCart;  // control signal
  desiredVelocityCart.Fill(0); //Initialize matrix with zeros
  Matrix<2> measurements;

  // Kalman filtering
  if(KalmanFilterEnabled()) { // only do this if Kalman Filter is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
     // UNCOMMENT AND MODIFY LINES BELOW TO IMPLEMENT THE KALMAN FILTER
     // Correction step

     measurements(0) = getPositionMotorA()*0.033; //transform encoders measurement (getPositionMotorA() and getPositionMotorB()) to cart position measurement
     measurements(1) = -getPendulumAngle();
     CorrectionUpdate(measurements, _xhat, _Phat, _nu, _S);     // do the correction step -> update _xhat, _Phat, _nu, _S

  
  }


  if(controlEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 0' in QRoboticsCenter)

    // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT THE FEEDFORWARD INPUTS (ASSIGNMENT 4.2, no state feedback here)
    // COMMENT OR REMOVE LINES BELOW ONCE YOU IMPLEMENT THE POSITION STATE FEEDBACK CONTROLLER
    
    // Compute the feedforward input of the cart
    // The feedforward is here returned by the built-in trajectory: trajectory.v()
    desiredVelocityCart(0) = trajectory.v();  //desired forward velocity of the cart (in m/s)
    // The trajectory must be started by pushing 'Button 2' in QRoboticsCenter, otherwise will return zeros
    // after any experiment the trajectory must be reset pushing 'Button 3' in QRoboticsCenter
    //
    // // apply the static transformation between velocity of the cart and velocity of the motors
    desiredVelocityMotorA = desiredVelocityCart(0)/0.033;        // calculate the angular velocity of the motor A using desiredVelocityCart
    desiredVelocityMotorB = desiredVelocityCart(0)/0.033;        // calculate the angular velocity of the motor B using desiredVelocityCart



    // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT POSITION CONTROLLER (ASSIGNMENT 4.4)
    // step reference in the desired pendulum mass position
    // ref(0)= readValue(0);         // r [m] - use channel 0 to provide the step reference
    
    // // State feedback controller
    // float arrayKfb[1][3]{{0.9590, -0.9577, 0.0570}};  // state feedback gain Kfb, to design
    // Matrix<1, 3> Kfb = arrayKfb;
    
    // // Compute feedback signal ufb = -Kfb*x
    // Matrix<1> ufb = -Kfb * _xhat;
    
    // // Feedforward controller
    // float arrayKff[1][1] = {{0.9590}};  // feedforward gain Kff, to design
    // Matrix<1> Kff = arrayKff;
    
    // Matrix<1> uff = Kff*ref;
    
    // // Compute the control action u = uff + ufb
    // float desiredVelocityCart = uff(0) + ufb(0);  // desired forward velocity of the cart from the feedforward and state feedback controller
    
    // float desiredVelocityMotorA = desiredVelocityCart/0.033;        // calculate the angular velocity of the motor A using desiredVelocityCart
    // float desiredVelocityMotorB = desiredVelocityCart/0.033;        // calculate the angular velocity of the motor B using desiredVelocityCart

    // // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT VELOCITY CONTROLLER
    float Va = getSpeedMotorA();
    float Vb = getSpeedMotorB();
    float eA = desiredVelocityMotorA - Va;  //  calculate the speed error of motor A (in radians/s)
    float eB = desiredVelocityMotorB - Vb;  
    
    float uA = 0.8736*eA - 0.8054*errorA[0] + controlA[0];   // equation (2), the difference equation   
    float uB = 0.8736*eB - 0.8054*errorB[0] + controlB[0];      // equation (2), the difference equation 
    errorA[0] = eA; errorB[0] = eB; controlA[0] = uA; controlB[0] = uB;

    volt_A = uA ;
    volt_B = uB ;

    //// COMMENT OR REMOVE LINES BELOW ONCE YOU IMPLEMENT THE VELOCITY CONTROLLER
    // volt_A = 0.0;
    // volt_B = 0.0;

    // Send wheel speed command
    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  }
  else                      // do nothing since control is disabled
  {
   desiredVelocityCart(0) = 0.0;
   setVoltageMotorA(0.0);
   setVoltageMotorB(0.0);
  }

  // Kalman filtering
  if(KalmanFilterEnabled()) { // only do this if Kalman Filter is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Prediction step
    PredictionUpdate(desiredVelocityCart, _xhat, _Phat);                    // do the prediction step -> update _xhat and _Phat
  }

  // Send useful outputs to QRC
  // to check functioning of trajectory and feedforward
  // writeValue(0, trajectory.v());
  // writeValue(1, trajectory.X());
  // writeValue(2, trajectory.hasMeasurements());
  // writeValue(3, getSpeedMotorA());
  // writeValue(4, getSpeedMotorB());
  // writeValue(5, volt_A);
  // writeValue(6, volt_B);
  

  // // Useful outputs to QRC for assignment questions
  writeValue(0,measurements(0) + 0.135*measurements(1));

  writeValue(1, _xhat(0));
  writeValue(2, _xhat(1));
  writeValue(3, _xhat(2));
  writeValue(4, _Phat(0,0));
  writeValue(5, _Phat(1,0));
  writeValue(6, _Phat(1,1));
  writeValue(7, _Phat(2,0));
  writeValue(8, _Phat(2,1));
  writeValue(9, _Phat(2,2));
  writeValue(10, measurements(0));
  writeValue(11, measurements(1));

  //triggers the trajectory to return the next values during the next cycle
  trajectory.update();
}

void Robot::resetController(){
  // Set all errors and control signals in the memory back to 0
  errorA[0] = 0.0;
  errorB[0] = 0.0;
  controlA[0] = 0.0;
  controlB[0] = 0.0;
}

void Robot::resetKalmanFilter() {
  // UNCOMMENT AND MODIFY LINES BELOW TO IMPLEMENT THE RESET OF THE KALMAN FILTER
  // Initialize state covariance matrix
  _Phat.Fill(0);       // Initialize the covariance matrix
  _Phat(0,0) = 1e-4;      // Fill the initial covariance matrix, you can change this according to your experiments
  _Phat(1,1) = 1e-6;
  _Phat(2,2) = 1e-6;

  // Initialize state estimate
  _xhat(0) = 0.0;       // Change this according to your experiments
  _xhat(1) = 0.0;
  _xhat(2) = 0.0;
  
  // Reset innovation and its covariance matrix
  _S.Fill(0);
  _nu.Fill(0);
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::KalmanFilterEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    resetController();
    message("Controller reset and enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Control disabled.");
  }
}

void Robot::button1callback() {
  if(toggleButton(1)){
      resetKalmanFilter();            // Reset the Kalman filter
      message("Kalman filter reset and enabled.");
  }
  else
  {
    message("Kalman filter disabled.");
  }
}

void Robot::button2callback() {
  if(toggleButton(2)) {
    trajectory.start();
    message("Trajectory started/resumed.");
  } else {
    trajectory.stop();
    message("Trajectory stopped.");
  }
}

void Robot::button3callback() {
     _button_states[2] = 0;
    trajectory.reset();
    message("Trajectory reset.");
}
