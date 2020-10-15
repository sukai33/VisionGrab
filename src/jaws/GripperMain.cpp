/**
 * @Author: PoplarTang
 * @CreateTime: 2019-12-24
 * @Description: 
 */


#include "Gripper.h"

int main(int argc, char **argv) {
    // sudo chmod 777 /dev/ttyUSB0
    Gripper gripper("/dev/ttyUSB0");

    gripper.gripperCatch(500, 200);

    usleep( 2 * 1000 * 1000);

    gripper.gripperRelease();

    return 0;
}
