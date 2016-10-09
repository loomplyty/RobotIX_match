#ifndef FREEGAIT_H
#define FREEGAIT_H

#include <aris.h>

#include <thread>
#include <functional>
#include <cstdint>
#include <map>


#include <Robot_Type_III.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>
#include <Basic_Gait.h>


#include "FreeGaitAlgorithms.h"
#include "FreeGaitBase.h"

namespace FreeGait
{

//thread
void startLogDataThread();

//parse functions and gait functions
void parseAdjustSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseIMU(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parsePitch(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseGoSlope35(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseGoSlopeVisionFast2(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

int GoSlopeByVisionFast2(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
int GoSlope35(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);





}


#endif // FREEGAIT_H
