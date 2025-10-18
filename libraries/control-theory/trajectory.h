#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#ifdef PENDULUM
  #if __has_include("data_pendulum.h")
    #define DATA_AVAILABLE
    #include "data_pendulum.h"
  #endif
#else
  #define SWIVEL // default to swivel
  #if __has_include("data_swivel.h")
    #define DATA_AVAILABLE
    #include "data_swivel.h"
  #endif
#endif

#include <stdint.h>
#include <avr/pgmspace.h>

#define _TRAJ_READ_FLOAT_IF(NAME, ID, COND) \
  float NAME() { \
    if(COND) return (float) ((int16_t) pgm_read_word_far(pgm_get_far_address(data_##ID) + (uint32_t) 2*_counter))/0x7FFF*data_##ID##max; \
    else return 0.0; \
  }

#define _TRAJ_READ_BOOL_IF(NAME, ID, COND) \
  bool NAME() { \
    if(COND) return (bool) pgm_read_word_far(pgm_get_far_address(data_##ID) + (uint32_t) 2*_counter); \
    else return false; \
  }

class Trajectory {
private:
  uint16_t _counter;
  bool _started;
  bool _stopped;

public:
#ifdef DATA_AVAILABLE
  Trajectory() : _counter(0), _started(false), _stopped(true) { }
  void start() {
    _started = true;
    _stopped = false;
  }
  void stop() {
    _stopped = true;
  }
  void update() {
    if(_started && !_stopped) {
      if(_counter < (NUMBER_OF_SAMPLES-1)) _counter++;
      else _stopped = true;
    }
  }
  void reset() {
    _counter = 0;
    _started = false;
    _stopped = true;
  }
#else // DATA_AVAILABLE
  Trajectory() : _counter(0), _started(false), _stopped(true) { }
  void start() { }
  void stop() { }
  void update() { }
  void reset() { }
#endif // DATA_AVAILABLE
  _TRAJ_READ_BOOL_IF(hasMeasurements, 0, _started);
  _TRAJ_READ_FLOAT_IF(v, 1, _started && !_stopped);
  _TRAJ_READ_FLOAT_IF(X, 2, _started);

  #ifdef SWIVEL
  _TRAJ_READ_FLOAT_IF(Y, 3, _started);
  _TRAJ_READ_FLOAT_IF(omega, 4, _started && !_stopped);
  _TRAJ_READ_FLOAT_IF(Theta, 5, _started);
  #endif // SWIVEL

};

#endif //TRAJECTORY_H
