
#include "Gripper.h"
#include <iostream>

using namespace std;
/// sudo chmod 777 /dev/ttyUSB0
Gripper::Gripper(char *port) : port(port) {
  rs232 = new kfx::RS232(port, 115200);
  if (!rs232->IsAvailable()) {
    cout << "serial port is not available" << endl;
    return;
  }
}

Gripper::~Gripper() {}

void Gripper::connect() {

}

void Gripper::disconnect() {
  rs232->Close();
}

void Gripper::gripperCatch(int speed, int power) {
  if (speed > 1000) {
    speed = 1000;
  }
  if (speed < 1) {
    speed = 1;
  }
  if (power > 1000) {
    power = 1000;
  }
  if (power < 30) {
    power = 30;
  }
  std::cout << "夹取： speed-" << speed << " power-"<< power << std::endl;

  int sendSize = 10;
  u_char *data = new u_char[sendSize];
  //B0-B1： 帧头
  data[0] = 0xEB;
  data[1] = 0x90;
  //B2： ID号
  data[2] = 0x01;
  //B3: 数据体长度(b4-b8)
  data[3] = 0x05;
  //B4: 指令号
  data[4] = 0x10;
  //B5-B8
  data[5] = speed & 0x00ff;
  data[6] = speed >> 8;
  data[7] = power & 0x00ff;
  data[8] = power >> 8;
  //B9(校验位 (b2 + b3 + ... + b8) & 0xFF)
  data[9] = (data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8]) & 0xFF;

  rs232->Write(data, sendSize);
}

void Gripper::gripperRelease(int speed) {
    if (speed > 1000) {
        speed = 1000;
    }
    if (speed < 1) {
        speed = 1;
    }

    std::cout << "放开： speed-" << speed << std::endl;
    int sendSize = 8;
    u_char *data = new u_char[sendSize];
    //B0-B1： 帧头
    data[0] = 0xEB;
    data[1] = 0x90;
    //B2： ID号
    data[2] = 0x01;
    //B3: 数据体长度(b4-b8)
    data[3] = 0x03;
    //B4: 指令号
    data[4] = 0x11;
    //B5-B8
    data[5] = speed & 0x00ff;
    data[6] = speed >> 8;
    //B9(校验位 (b2 + b3 + ... + b8) & 0xFF)
    data[7] = (data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF;

    rs232->Write(data, sendSize);
}

void Gripper::getEGParam() {}