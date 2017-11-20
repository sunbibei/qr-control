
#ifndef QR_CONTROL_ULTILITY_H_
#define QR_CONTROL_ULTILITY_H_

#include <string>
#include <iostream>
#include <cmath>
static const  int Joint_Num = 12;
static const float PI = 3.14159265;
static const  int Foot_Steps = 10;
static const  int Swing_Height =6;
static const  int Stance_Height = 46;
static const  int Init_Num = 20;
static const  int Swing_Num  = 30;
static const  int Update_Rate = 1000;
static const double L0 = 4;
static const double L1 = 27.3;
static const double L2 = 22.5;
static const double Body_L = 27.65;
static const double Body_W = 16.8;
static const double Body_D = 10;
static const int cogThreshold = 5;
static int Stance_Num = 40;
static bool lb_flag =true;
static bool lf_flag =true;
static bool rb_flag =true;
static bool rf_flag =true;

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
