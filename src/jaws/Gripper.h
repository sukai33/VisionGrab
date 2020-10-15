
#ifndef LASER_PKG_SRC_GRIPPER_H_
#define LASER_PKG_SRC_GRIPPER_H_

#include "rs232.h"

class Gripper {

 public:
  Gripper(char *port);
  ~Gripper();

 private:
  char *port;
  kfx::RS232 *rs232;

 public:
  void connect();
  void disconnect();
  void gripperCatch(int speed = 500, int power = 100);
  void gripperRelease(int speed = 500);
  void getEGParam();

};

#endif //LASER_PKG_SRC_GRIPPER_H_
