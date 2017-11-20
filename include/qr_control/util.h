#pragma once

#include <glog/logging.h>

#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"

const int JNT_NUM = 12;
const int HEIGHT = 45;

const int FOOT_STEP = 10;
const int Swing_Height = 4;

const int Init_Num = 500;
const int Swing_Num  = 5000;
const int COG_COUNT = 5000;
const int Update_Rate = 1000;

const double L0 = 4;
const double L1 = 27.3;
const double L2 = 22.5;

const double BODY_LENGTH = 20;  // 595 mm
const double BODY_WIDTH = 13.5; // 336 mm

const float PI = 3.14159265;

inline int Sgn(int a) {
  return (a > 0 ? 1 : -1);
}

struct __Commands {
        double position_;     // Last commanded position
        double velocity_;     // Last commanded velocity
        bool   has_velocity_; // false if no velocity command has been specified
};

struct __Position {
        double x;
        double y;
        double z;
};

struct __Angle_Leg {
        double pitch;
        double hip;
        double knee;
};

typedef struct Position {
        __Position lf;
        __Position rf;
        __Position lb;
        __Position rb;

}Position, *Position_Ptr;

typedef struct Angle {
        __Angle_Leg lf;
        __Angle_Leg rf;
        __Angle_Leg lb;
        __Angle_Leg rb;
}Angle, *Angle_Ptr;
