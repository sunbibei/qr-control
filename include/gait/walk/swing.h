#ifndef QR_CONTROL_SWING_H_
#define QR_CONTROL_SWING_H_

#include <cmath>
#include <iostream>

#include "ultility.h"

namespace qr_control {

  class Swing {
  private:
    bool verbose;


  public:     
    Swing();
    void init();
    _Position get_rect_pos(_Position Start_point, _Position End_point, int Loop, int T, int H);
    _Position get_eclipse_pos(_Position Start_point, _Position End_point, int Loop,int T, int H);
    _Position compoundCycloidPosition(_Position Start_point, _Position End_point, int Loop,int T, int H);
    EV3 compoundCycloidVelocity(_Position Start_point, _Position End_point, int Loop, int T, int H);

};
}
#endif

