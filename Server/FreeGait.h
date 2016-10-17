#ifndef FREEGAIT_H
#define FREEGAIT_H

#include "aris.h"
#include "Robot_Type_III.h"
#include <Robot_Base.h>
#include <Robot_Gait.h>
#include <Basic_Gait.h>


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
    single=1,
    multiDiscrete=2,
    multiContinuous=3,
};
enum LegPhase
{
    swing=1,
    stance=2,
};
struct FreeGaitParams :public aris::server::GaitParamBase
{
    std::int32_t totalCount=3000;
    double n=1;
    double d=0.1;
    double h=0.08;
    double a=0;
    double b=0;
    double l=0;
    int m=GaitMode::single;
};
struct RobotData
{
    double waist;
    double bodyPee[6];
    double legPee[18];
    double imu[3];
    double legForce[36];
    int legPhase[6];
    char CS='C';//'G' or 'B' or 'C'
    double motorCount[18];
    double motorCurrent[18];
};


//***thread***//
void startLogDataThread();

//***parse functions and gait functions***//
//adjust parameters
//void parseFreeGaitAdjust(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

//sensor switch
void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseIMU(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

// parse functions
void parseGoFreeGait(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
// gait functions
int GoFreeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

struct MotionStatus
{



};


class FreeGait
{//c for current; n for next
public:
    //model
  //  Robots::RobotTypeIII &robot;
  //  FreeGaitParams &params;
    //gait param
    int g_gaitCount;
    int g_stepNumFinished;
    int g_countPerStep;
    bool g_isGaitFinished;

    //step param
    bool s_isStepFinished;
    int s_swingID[3]{1,5,3};
    int s_stanceID[3]{0,2,4};
    double s_dutyFactor{0.6};
    double s_beginBodyPee[6];
    double s_beginLegPee[18];
    double s_endBodyPee[6];
    double s_endLegPee[6];

    //count param
    double c_bodyPee[6];
    double c_legPee[18];
    double c_planBodyPee[6];
    double c_planLegPee[18];
    double c_planInterval{10};//10ms for example

    //double transAcc[3]{0.1,0.1,0.1};
    //double rotAcc[3]{0.1,0.1,0.1};

    FreeGait();
    ~FreeGait();
    void updateRobot(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
    void switchLegs();
    void setDutyFactor(const double factor);

    //void setBodyAcc(const double *transAcc, const double *rotAcc);

};


}


#endif // FREEGAIT_H
