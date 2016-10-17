#include "FreeGait.h"
#include <math.h>

namespace FreeGait
{
static bool isForceUsed=false;
static bool isIMUUsed=false;
static bool isVisionUsed=false;

aris::control::Pipe<RobotData> logPipe(true);

void startLogDataThread()
{
    static auto logThread = std::thread([]()
    {
        sleep(1);
        //aris::core::logFileNames();
        std::cout << "Sleep for a second waiting for something to init\n";

        std::time_t beginTime{};
        time(&beginTime);

        auto timeinfo = localtime(&beginTime);
        char timeCh[1024]={0};//???0

        strftime(timeCh,1024,"_%Y-%m-%d_%H-%M-%S_log.txt",timeinfo);
        std::string filename = "data" + string(timeCh);

        static std::ofstream file;
        file.open(filename);


        while(true)
        {

            RobotData data;
            logPipe.recvInNrt(data);

            file << std::setprecision(15);
            file<<data.waist<<" ";

            for (int j = 0; j < 3; j++)
            {
                file << data.imu[j] << "   ";
            }

            for (int j = 0; j < 6; j++)
            {
                file << data.bodyPee[j] << "   ";
            }

            for (int j = 0; j < 18; j++)
            {
                file << data.legPee[j] << "   ";
            }
            for (int j = 0; j < 36; j++)
            {
                file << data.legForce[j] << "   ";
            }
            for (int j = 0; j < 6; j++)
            {
                file << data.legPhase[j] << "   ";
            }
            for (int j = 0;j < 18; j++)
            {
                file<<data.motorCount[j]<<" ";
            }
            for (int j = 0;j < 18 ; j++)
            {
                file<<data.motorCurrent[j]<<" ";
            }

            file << std::endl;

        }
        file.close();
    });
}


void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isForceUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isForceUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

void parseIMU(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isIMUUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isIMUUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

void parseVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isVisionUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isVisionUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

void parseGoFreeGait(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    FreeGaitParams param;
    for (auto &i : params)
    {
        if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "lateral")
        {
            param.l=stod(i.second);
        }
        else if (i.first == "totalCount")
        {
            param.totalCount=stoi(i.second);
        }

        else if (i.first == "mode")
        {
            param.m= stoi(i.second);
        }
    }
    msg.copyStruct(param);
    cout<<"parse finished"<<endl;

}
int GoFreeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot=static_cast<Robots::RobotTypeIII &>(model);
    auto &param=static_cast<const FreeGaitParams &>(param_in);

    static aris::dynamic::FloatMarker beginMak{robot.ground()};
    static aris::dynamic::FloatMarker InitMak{robot.body()};
    static FreeGait fg;

    if(param.count==0)
    {
        rt_printf("Gait begins!\n");
        fg.g_stepNumFinished=0;
        fg.g_gaitCount=0;
        fg.s_isStepFinished=false;
    }

    return 1;
}

FreeGait::FreeGait()
{


}
FreeGait::~FreeGait()
{

}
void FreeGait::updateRobot(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
 //   robot=static_cast<Robots::RobotTypeIII &>(model);
 //   params=static_cast<const FreeGaitParams &>(param_in);
}


}
