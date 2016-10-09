#ifndef FREEGAITALGORITHMS_H
#define FREEGAITALGORITHMS_H
#include "FreeGaitBase.h"

namespace FreeGait {
namespace Algorithms {

//***Planning***//
void GetBodyOffset(const double pitch, const double roll, double* offset);
void GetBodyOffsetRobotIX(const double pitch, const double roll, double* offset);
void GetPlaneFromStanceLegs(const double* stanceLegs,double* normalVector);


}
}

















#endif // FREEGAITALGORITHMS_H
