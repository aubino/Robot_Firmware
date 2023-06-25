#include "encoders.h"
#include "Rotary.h"

Wheel left_wheel(0,1,2,3,4,5) ;/*, right_wheel(0,1,2,3,4,5)*/ 

void on_change_left() { left_wheel._on_change() ;}

void update_left_wheel_speed() { left_wheel._update_speed() ; }

