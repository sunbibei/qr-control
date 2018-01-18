#include "qr_control/swing.h"

using namespace qr_control;

Swing::Swing()
{}

void Swing::init()
{
  verbose = false;
  fast_phase = 1;
}

/* 
compound Cycloid Position:
  p.x, p.y = L * (t/T - 1/2/pi*sin(2*pi*t/T));
  p.z = 2 * H * (t/T - 1/4/PI*sin(4*PI*t/T));
  p.z = 2 * H * ((T-t)/T - 1/4/PI*sin(4*PI*(T-t)/T));
*/
_Position Swing::compoundCycloidPosition(_Position Start_point, _Position End_point, int Loop, int T, int H)
{
  _Position p = {0,0,0};

  p.x = (End_point.x - Start_point.x) * ((float)Loop/(float)T - 1.0/2.0/PI*sin(2.0*PI*Loop/T));  
  p.y = (End_point.y - Start_point.y) * ((float)Loop/(float)T - 1.0/2.0/PI*sin(2.0*PI*Loop/T));

  if(Loop<=T/2)
  {
    p.z = 2.0 * H * ((float)Loop/(float)T - 1.0/4.0/PI*sin(4.0*PI*Loop/T));
    // std::cout<<"Loop<T/2    "<<p.z<<std::endl;
  }
  else if(Loop<=T)
  {
    p.z = 2.0 * H * ((float)(T-Loop)/(float)T - 1.0/4.0/PI*sin(4.0*PI*(T-Loop)/T));
    
    // std::cout<<"Loop<T    "<<p.z<<std::endl;
  }

  if(verbose)
  {
    std::cout<<"Swing class:"<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
  }
  return p;
}

/* 
compound Cycloid Velocity:
  p.x, p.y = L * (1/T - 1/T*cos(2*pi*t/T));
  p.z = 2 * H * (1/T - 1/4/PI*sin(4*PI*t/T));
  p.z = 2 * H * ((T-t)/T - 1/4/PI*sin(4*PI*(T-t)/T));
*/
EV3 Swing::compoundCycloidVelocity(_Position Start_point, _Position End_point, int Loop, int T, int H)
{
  EV3 v ;

  v(0) = (End_point.x - Start_point.x) * (1.0/(float)T - 1.0/(float)T*cos(2.0*PI*Loop/T));  
  v(1) = (End_point.y - Start_point.y) * (1.0/(float)T - 1.0/(float)T*cos(2.0*PI*Loop/T));

  if(Loop<=T/2)
  {
    v(2) = 2.0 * H * (1.0/(float)T - 1.0/(float)T*cos(4.0*PI*Loop/T));
  }
  else if(Loop<=T)
  {
    v(2) = 2.0 * H * (-1.0/(float)T + 1.0/(float)T*cos(4.0*PI*(T-Loop)/T));
  }

  if(verbose)
  {
    std::cout<<"Swing class:"<<v(0)<<" "<<v(1)<<" "<<v(2)<<std::endl;
  }
  return v;
}

int Swing::getSwingPhase()
{
  return fast_phase;
}

_Position Swing::fast(_Position Start_point, _Position End_point, _Position curr_pos, int Loop,int T, int H)
{
  float beta = 0.1;
  _Position p = {0,0,0};

  if(fast_phase == 1)
  {
    p = Start_point;
    p.z = p.z + beta * Loop;

    if(fabs(Start_point.x - curr_pos.x) < 0.3 
      && fabs(Start_point.y - curr_pos.y) < 0.3 
      && fabs(Start_point.z + H - curr_pos.z) < 0.3)
    {
      fast_phase = 2;
    }
  }
  else if(fast_phase == 2)
  {
    p = End_point;
    p.z = p.z + H;

    if(fabs(p.x - curr_pos.x) < 0.3 
      && fabs(p.y - curr_pos.y) < 0.3 
      && fabs(p.z - curr_pos.z) < 0.3)
    {
      fast_phase = 3;
    }
  }
  else if(fast_phase == 3)
  {
    p = curr_pos;
    p.z -= beta;
  }

return p;
}


_Position Swing::get_rect_pos(_Position Start_point, _Position End_point, int Loop, int T, int H)
{

  _Position p = {0,0,0};
  float a = (End_point.x - Start_point.x)/(float)T*3;
  float b = (End_point.y - Start_point.y)/(float)T*3;
  float c = (float)H/(float)T*3;
  // std::cout<<"Swing class:"<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;

  if(Loop<=T/3)
  {
    p.z = Loop * c;
    // std::cout<<"Loop<T/2    "<<p.z<<std::endl;
  }
  else if(Loop<=T/3*2)
  {
    p.x = (Loop-T/3) * a;
    p.y = (Loop-T/3) * b;
    p.z = T/3 * c;    
  }
  else if(Loop<=T)
  {
    c = (float)(End_point.z - Start_point.z - H)/(float)T*3;
    p.x = T/3 * a;
    p.y = T/3 * b;
    p.z = H + (Loop-T/3*2) * c;  
    // std::cout<<"Loop<T    "<<p.z<<std::endl;
  
  }

  if(verbose)
  {
    std::cout<<"Swing class:"<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
  }

  return p;
}

_Position Swing::get_eclipse_pos(_Position Start_point, _Position End_point, int Loop,int T, int H)
{
  _Position p = {0,0,0};
  int sgn = Sgn(End_point.x - Start_point.x);
  float angle = PI - PI * Loop/T;
  float centre = (Start_point.x + End_point.x)/2;
  float a =  fabs(Start_point.x - End_point.x)/2;
  float b = H;
  p.x = sgn*(a + a*cos(angle));
  p.z = b*sin(angle);

  sgn = Sgn( End_point.y - Start_point.y);
  a =  fabs(Start_point.y - End_point.y)/2;

  p.y = sgn*(a + a*cos(angle));

  if(verbose)
  {
    std::cout<<"Swing class:"<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
  }
  return p;
}