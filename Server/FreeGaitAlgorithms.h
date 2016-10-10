#ifndef FREEGAITALGORITHMS_H
#define FREEGAITALGORITHMS_H
#include "FreeGaitBase.h"

namespace FreeGait {
namespace Algorithms {

//***Motion***//
void GetBodyOffset(const double pitch, const double roll, double* offset);
void GetBodyOffsetRobotIX(const double pitch, const double roll, double* offset);
void GetTM_C_2_B(double* Legs_2_b,double *TM_c_2_b);// need more considerations on how to determine

//***Trajectory***//


//***Force***//


//***Vision***//


}
}

















#endif // FREEGAITALGORITHMS_H
