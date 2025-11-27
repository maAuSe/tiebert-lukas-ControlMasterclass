#ifndef ROBOT_H
#define ROBOT_H

/*
 * ROBOT Class
 *
 * Class incorporating the robot. This class is used to define state machines,
 * control algorithms, sensor readings,...
 * It should be interfaced with the communicator to send data to the world.
 *
 */

#include "mecotron.h" // Include MECOTRON header

class Robot : public MECOtron {
  private:

      float errorA_prev = 0.0;
      float errorB_prev = 0.0;
      float controlA_prev	 = 0.0;
      float controlB_prev = 0.0;

      const float n0d0 = 0.4761;
      const float n1d0 = -0.4721;
      const float d1d0 =  -1;

  public:
    // Constructor
    Robot() { }

    void control();

    // General functions
    bool init();  // Set up the robot

    bool controlEnabled();

    void button0callback();
    void button1callback();

    // Controller related functions
    void resetController(); 

};

#endif // ROBOT_H
