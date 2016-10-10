#include "FreeGait.h"
#include <math.h>


namespace FreeGait
{
static bool isForceUsed=false;
static bool isIMUUsed=false;
static bool isVisionUsed=false;



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

}
