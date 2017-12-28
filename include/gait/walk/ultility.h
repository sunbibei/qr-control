
#ifndef QR_CONTROL_ULTILITY_H_
#define QR_CONTROL_ULTILITY_H_

#include <string>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

#define EV2 Eigen::Vector2d
#define EV3 Eigen::Vector3d
#define EVX Eigen::VectorXd
#define EM3 Eigen::Matrix3d
#define EMX Eigen::MatrixXd



static const  int Joint_Num = 12;
static const  int Leg_Num = 4;
static const float PI = 3.14159265;
static const  int Foot_Steps = 10;
static const  int Swing_Height = 5;
static const  int Stance_Height = 46;
static const  int Init_Num = 5;
static const  int Swing_Num  = 30;
static const  int Update_Rate = 100;

static const double L0 = 4;
static const double L1 = 27.3;
static const double L2 = 22.5;

static const double Body_L = 27.65;
static const double Body_W = 16.8;
static const double Body_D = 10;
static const float cogThreshold = 6.5;
static int Stance_Num = 10;
static bool lb_flag = true;
static bool lf_flag = true;
static bool rb_flag = true;
static bool rf_flag = true;

static const float YAW_MAX = 0.4165;
static const float YAW_MIN = -0.3295;

static const float HIP_MAX = 0.8733;
static const float HIP_MIN = -0.2629;

static const float KNEE_MAX = 1.4137;
static const float KNEE_MIN = 0.5952;

// enum Leg{LF,RF,LB,RB};
enum Leg{LF,RF,LB,RB};

inline int Sgn(int a)
{
	return (a>0 ? 1 : -1);
}
inline float max(float a, float b)
{
  return ((a>b)? a:b);
}
inline float min(float a, float b)
{
  return ((a>b)? b:a);
}

struct Commands
{
	double position_; // Last commanded position
	double velocity_; // Last commanded velocity
	bool has_velocity_; // false if no velocity command has been specified
  Commands( double position=0, double velocity=0, bool has_velocity=true) 
  : position_(position), velocity_(velocity), has_velocity_(has_velocity){} 
};

struct IMU
{	
	double pitch;
	double yaw;
	double roll;

  IMU( double pitch=0, double yaw=0, double roll=0) : pitch(pitch), yaw(yaw), roll(roll){}    
  
  IMU operator = ( const IMU& A )  
  {  
    return IMU(A.pitch, A.roll, A.yaw);  
  }  
};


struct Quartic
{
	double lf;
	double rf;
	double lb;
	double rb;
	
  Quartic( double lf=0, double rf=0, double lb=0, double rb=0) : lf(lf), rf(rf), lb(lb), rb(rb){}  

  int sum()
  {
    return (lf+rf+lb+rb);
  }

  Quartic operator + ( const Quartic& A )  
  {  
    return Quartic(this->lf+A.lf, this->rf+A.rf, this->lb+A.lb, this->rb+A.rb);  
  }  
  Quartic operator - ( const Quartic& A)  
  {  
    return Quartic(this->lf-A.lf, this->rf-A.rf, this->lb-A.lb, this->rb-A.rb);  
  } 
};

struct _Position
{
	double x;
	double y;
	double z;

  _Position( double x=0, double y=0, double z=0) : x(x), y(y), z(z){}    

  _Position operator + ( const _Position& A )  
  {  
    return _Position(this->x+A.x, this->y+A.y, this->z+A.z);  
  }  
  _Position operator - ( const _Position& A )  
  {  
    return _Position(this->x-A.x, this->y-A.y, this->z-A.z);  
  }  
  // _Position operator = ( const _Position& A )  
  // {  
  //   return _Position(A.x, A.y, A.z);  
  // }  
  void assign(double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }

};

struct _Angle_Leg
{
	double pitch;
	double hip;
	double knee;
};

typedef struct Position
{
	_Position lf;
	_Position rf;
	_Position lb;
	_Position rb;

  Position( _Position lf, _Position rf, _Position lb, _Position rb) : lf(lf), rf(rf), lb(lb), rb(rb){} 
  Position( const Position& p) : lf(p.lf), rf(p.rf), lb(p.lb), rb(p.rb){}    


} Position,*Position_Ptr;

typedef struct Angle
{
	_Angle_Leg lf;
	_Angle_Leg rf;
	_Angle_Leg lb;
	_Angle_Leg rb;
} Angle,*Angle_Ptr;

#endif
