#ifndef KUBOT_DATA_HOLDER_H_
#define KUBOT_DATA_HOLDER_H_

#include <string.h>

#pragma pack(1)

using namespace std;

typedef int int32;
typedef short int16;
typedef unsigned short unint16;

struct Controller_firmware {
   char version[16]; //版本
   char time[16];    //建立時間
};

struct Controller_led_status {
   short ledNum;            //LED編號
   short led_brightness;    //LED亮度
   short led_speed;         //LED閃爍速度
   short led_color_r;       //LED顏色 R
   short led_color_g;       //LED顏色 G
   short led_color_b;       //LED顏色 B
   short led_mode;          //LED亮的模式
};

struct Controller_servo_status {
   short servoNum;      // 伺服舵機編號
   short servo_angle;    // 伺服舵機旋轉角度
};

#pragma pack(0)

class Data_holder {
public:
   static Data_holder* get() {
      static Data_holder dh;
      return &dh;
   }

private:
   Data_holder()
   {
      memset(&firmware_info, 0, sizeof(struct Controller_firmware));
      memset(&led_status, 0, sizeof(struct Controller_led_status));
      memset(&servo_status, 0, sizeof(struct Controller_servo_status));
   }

public:
   struct Controller_firmware       firmware_info;
   struct Controller_led_status     led_status;
   struct Controller_servo_status   servo_status;
};

#endif /* KUBOT_DATA_HOLDER_H_ */