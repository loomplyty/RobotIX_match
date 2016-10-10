#ifndef FREEGAIT_H
#define FREEGAIT_H

#include <aris.h>

//#include <thread>
//#include <functional>
//#include <cstdint>
//#include <map>


#include <Robot_Type_III.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>
#include <Basic_Gait.h>


#include "FreeGaitAlgorithms.h"
#include "FreeGaitBase.h"

namespace FreeGait
{

enum GaitMode
{
    Single=1,
    MultiDiscrete=2,
    MultiContinuous=3
};
struct FreeGaitParams :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
    double n{1};
    double d{ 0.1 };
    double h{ 0.08 };
    double a{ 0 };
    double b{ 0 };
    double l{0};
    int m{GaitMode::Single};
 };



//***thread***//
void startLogDataThread();

//***parse functions and gait functions***//
//adjust parameters
void parseFreeGaitAdjust(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

//sensor switch
void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseIMU(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

// parse functions
void parseGoFreeGait(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
// gait functions
int GoFreeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

}


#endif // FREEGAIT_H
