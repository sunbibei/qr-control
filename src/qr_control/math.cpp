#include "qr_control/math.h"

using namespace qr_control;

Math::Math()
{}

void Math::init()
{
  verbose = true;
}

//三角形面积S=√[p(p-a)(p-b)(p-c)],其中p=(a+b+c)/2.(海伦公式)内切圆半径r=2S/(a+b+c)外接圆半径R=abc/4S
float Math::inscribedCircleRadius(const _Position &A, const _Position &B, const _Position &C)
{
  float c = sqrt(pow((A.x-B.x),2) + pow((A.y-B.y),2));
  float b = sqrt(pow((A.x-C.x),2) + pow((A.y-C.y),2));
  float a = sqrt(pow((C.x-B.x),2) + pow((C.y-B.y),2));
  float p = (a+b+c)/2;
  float s = sqrt(p*(p-a)*(p-b)*(p-c));
  return 2*s/(a+b+c);
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating forward kinematics for quadruped robot(3 DOF) using D-H methods
   Formula:
   Px = L1 * S1 + L2 * S12;
   Py = L0 * S0 + L1 * S0 * C1 + L2 * S0 * C12;
   Pz = - Lo * C0 - L1 * C0 * C1 - L2 * C0 * C12;
**************************************************************************/
_Position Math::cal_formula(const _Angle_Leg &A)
{
  _Position p;
  p.x = L1 * sin(A.hip) + L2 * sin(A.hip + A.knee);
  p.y = L0 * sin(A.pitch) + L1 * sin(A.pitch) * cos(A.hip) + L2 * sin(A.pitch) * cos(A.hip + A.knee);
  p.z = - L0 * cos(A.pitch) - L1 * cos(A.pitch) * cos(A.hip) - L2 * cos(A.pitch) * cos(A.hip + A.knee);
  return p;
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating reverse kinematics for quadruped robot(3 DOF)
   Formula:
   Theta_0 = atan(- Py / Pz );
   Theta_1 = 2 * atan((Epsilon + sqrt(Epsilon^2 - Phi * (L2 * S2 - Delte))) / Phi);
   Theta_2 = sgn * acos((Delte^2 + Epsilon^2 - L1^2 - L2^2) / 2 / L1 / L2);
   Meantime, Delte = Px ; Phi = Delte + L2 * S2; Epsilon = L0 + Pz * C0 - Py * S0;
**************************************************************************/
_Angle_Leg Math::cal_kinematics(const _Position &P,int Sgn)
{
  _Angle_Leg A= {0,0,0};

  double Delte = -P.x;
  A.pitch = atan(-P.y / P.z);

  double Epsilon = L0 + P.z * cos(A.pitch) - P.y * sin(A.pitch);
  A.knee = Sgn * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2)) / 2.0 / L1 / L2);

  double Phi = Delte + L2 * sin(A.knee);
  if(Phi == 0)
  {
    Phi = Phi + 0.000001;
  }
  A.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(A.knee) - Delte))) / Phi);

  return A;
}
//ratio = BP/(BP+BA)
_Position Math::formulaLineSection(const _Position &A, const _Position &B, float ratio)
{
  _Position result;
  result.x = B.x - ratio * (B.x - A.x);
  result.y = B.y - ratio * (B.y - A.y);
  return result;
}

_Position Math::getInnerHeart(const _Position &A, const _Position &B, const _Position &C)
{ 
  double a = 0, b = 0, c = 0;
  _Position heart;

  a = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2));
  b = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
  c = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));

  heart.x = (a * A.x + b * B.x + c * C.x ) / (a + b + c);
  heart.y = (a * A.y + b * B.y + c * C.y ) / (a + b + c);

  std::cout<<"Math class: innerheart:"<<heart.x<<","<<heart.y<<std::endl;

  return heart;
}

_Position Math::getCrossPoint(const _Position &A_start, const _Position &A_end, const _Position &B_start, const _Position &B_end)
{ 
  double k1, k2, b1, b2;
  _Position cross_point = {0,0,0};

  if (A_start.x == A_end.x)
  {
    if (B_start.x == B_end.x)
    {
      std::cout<<"Error! 11"<<std::endl;
    }
    else if (B_start.y == B_end.y)
    {
      cross_point.x = A_start.x;
      cross_point.y = B_start.y;
    }
    else
    {
      k2 = (B_start.y - B_end.y )/(B_start.x - B_end.x);
      b2 = B_start.y - k2 * B_start.x;

      cross_point.x = A_start.x;
      cross_point.y = k2 * A_start.x + b2;
    }
  }
  else if (A_start.y == A_end.y)
  {
    if (B_start.x == B_end.x)
    {
      cross_point.x = A_start.y;
      cross_point.y = B_start.x;
    }
    else if (B_start.y == B_end.y)
    {
      std::cout<<"Error! 22"<<std::endl;    
    }
    else
    {
      k2 = (B_start.y - B_end.y )/(B_start.x - B_end.x);
      b2 = B_start.y - k2 * B_start.x;

      cross_point.x = (A_start.y - b2) / k2;
      cross_point.y = A_start.y;
    }
  }
  else
  {
    k1 = (A_start.y - A_end.y )/(A_start.x - A_end.x);
    b1 = A_start.y - k1 * A_start.x;

    if (B_start.x == B_end.x)
    {
      cross_point.x = B_start.x;
      cross_point.y = k1 * B_start.x + b1;
    }
    else if (B_start.y == B_end.y)
    {
      cross_point.x = (B_start.y - b1) / k1;
      cross_point.y = B_start.y;
    }
    else
    {
      k2 = (B_start.y - B_end.y )/(B_start.x - B_end.x);
      b2 = B_start.y - k2 * B_start.x;
      if (k1==k2)
        std::cout<<"Error! 33"<<std::endl;
      else
      {
        cross_point.x = (b1-b2)/(k2-k1);
        cross_point.y = k1 * (b1-b2)/(k2-k1) + b1;
      }
    }
  } 
 
  return cross_point;
}

