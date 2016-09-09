#pragma once
#include <Robot_Type_III.h>
#include <Basic_Gait.h>
#ifdef UNIX
#include "rtdk.h"
#endif

using namespace Robots::Gait;

struct ForceRecoverParam final :public aris::server::GaitParamBase
{
    std::int32_t recover_count{ 5000 };
    std::int32_t align_count{ 10000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{ 0.01 };//meter
    double alignPee[18]
    {   -0.33,   -0.85,   -0.65,
        -0.48,   -0.85,   0,
        -0.33,   -0.85,   0.65,
         0.33,   -0.85,   -0.65,
         0.48,   -0.85,   0,
         0.33,   -0.85,   0.65 };
    double recoverPee[18]
    {   -0.33,   -1.13,   -0.65,
        -0.48,   -1.13,   0,
        -0.33,   -1.13,   0.65,
         0.33,   -1.13,   -0.65,
         0.48,   -1.13,   0,
         0.33,   -1.13,   0.65 };
    bool is_zeroing_required {false};

    double force_threshold_y=200;
    double alignPeeY=0.85;
};


auto ForceRecoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

auto forceRecoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;

