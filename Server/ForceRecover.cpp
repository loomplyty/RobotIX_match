#include "ForceRecover.h"


auto ForceRecoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    ForceRecoverParam param;

    param.if_check_pos_min = false;
    param.if_check_pos_max = false;

    for (auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_leg, 6, true);
            std::fill_n(param.active_motor, MOTOR_NUM, false);
            std::fill_n(param.active_motor, 18, true);
        }
        else if (i.first == "first")
        {
            param.active_leg[0] = true;
            param.active_leg[1] = false;
            param.active_leg[2] = true;
            param.active_leg[3] = false;
            param.active_leg[4] = true;
            param.active_leg[5] = false;
            std::fill_n(param.active_motor, MOTOR_NUM, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            param.active_leg[0] = false;
            param.active_leg[1] = true;
            param.active_leg[2] = false;
            param.active_leg[3] = true;
            param.active_leg[4] = false;
            param.active_leg[5] = true;
            std::fill_n(param.active_motor, MOTOR_NUM, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if (i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);
            std:cout<<"parse leg_id:"<<leg_id<<std::endl;

            if (leg_id < 0 || leg_id>5)throw std::runtime_error("invalide param in parseRecover func");

            std::fill_n(param.active_leg, 6, false);
            param.active_leg[leg_id] = true;
            std::fill_n(param.active_motor, MOTOR_NUM, false);
            std::fill_n(param.active_motor + leg_id * 3, 3, true);


        }
        else if (i.first == "t1")
        {
            param.recover_count = std::stoi(i.second);
        }
        else if (i.first == "t2")
        {
            param.align_count = std::stoi(i.second);
        }
        else if (i.first == "margin_offset")
        {
            param.margin_offset = std::stod(i.second);
        }
        else if (i.first == "require_zero")
        {
            param.is_zeroing_required = std::stoi(i.second) == 0 ? false : true;
        }
        else if(i.first =="force_thr_y")
        {
            param.force_threshold_y=std::stod(i.second);
        }
        else if(i.first == "align_pee_y")
        {
            param.alignPeeY=std::stod(i.second);
        }
        else
        {
            throw std::runtime_error("unknown param in parseRecover func");
        }
    }
    for(int i=0;i<6;i++)
    {
        if(param.active_leg[i])
        {
            param.alignPee[i*3+1]=param.alignPeeY;
        }
    }

    msg_out.copyStruct(param);
}

auto forceRecoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const ForceRecoverParam &>(plan_param);

    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();



    static double beginPin[18], beginPee[18], alignPin[18],contactPee[18];

    static double realAlignPee[18],realRecoverPee[18];

    static double beginWa;

    static bool isContact[6];

    if (param.count == 0)
    {
        std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        robot.GetPee(beginPee, robot.body());
        robot.GetWa(beginWa);

        std::copy_n(param.alignPee, 18, realAlignPee);

        std::copy_n(param.recoverPee, 18, realRecoverPee);


        double l = realRecoverPee[8] - realRecoverPee[5];
        double h = l*std::sin(beginWa);
        double d = l*std::cos(beginWa);

        realAlignPee[1] += h*0.8;
        realAlignPee[2] = -d;
        realAlignPee[10] += h*0.8;
        realAlignPee[11] = -d;
        realAlignPee[7] += -h*0.8;
        realAlignPee[8] = d;
        realAlignPee[16] += -h*0.8;
        realAlignPee[17] = d;
        realRecoverPee[1] += h;
        realRecoverPee[2] = -d;
        realRecoverPee[10] += h;
        realRecoverPee[11] = -d;
        realRecoverPee[7] += -h;
        realRecoverPee[8] = d;
        realRecoverPee[16] += -h;
        realRecoverPee[17] = d;

        std::copy_n(realAlignPee, 18, contactPee);

        for(int i=0;i<6;i++)
        {
            isContact[i]=false;
        }






        const double pe[6]{ 0 };
        robot.SetPeb(pe);
        robot.SetWa(beginWa);
        //robot.SetPee(param.alignPee);
        robot.SetPee(realAlignPee);

        robot.GetPin(alignPin);
        robot.SetPee(beginPee, robot.body());

        //for test
        rt_printf("beginWa: %f\n", beginWa);
    }

    int leftCount = param.count < param.recover_count ? 0 : param.recover_count;
    int rightCount = param.count < param.recover_count ? param.recover_count : param.recover_count + param.align_count;

    double s = -(PI / 2)*cos(PI * (param.count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

    for (int i = 0; i < 6; ++i)
    {
        if (param.active_leg[i])
        {
            double pEE[3];
            if (param.count < param.recover_count)
            {
                for (int j = 0; j < 3; ++j)
                {
                    robot.motionPool().at(i * 3 + j).setMotPos(beginPin[i * 3 + j] * (cos(s) + 1) / 2 + alignPin[i * 3 + j] * (1 - cos(s)) / 2);
                }
            }
            else
            {

                for (int j = 0; j < 3; ++j)
                {
                    //pEE[j] = param.alignPee[i * 3 + j] * (cos(s) + 1) / 2 + param.recoverPee[i * 3 + j] * (1 - cos(s)) / 2;
                    pEE[j] = realAlignPee[i * 3 + j] * (cos(s) + 1) / 2 + realRecoverPee[i * 3 + j] * (1 - cos(s)) / 2;
                }
                robot.SetWa(beginWa);
                robot.pLegs[i]->SetPee(pEE);
            }



            if (param.is_zeroing_required && param.count == param.recover_count)
            {
                param.force_data->at(i).isZeroingRequested = true;
            }
            else if(param.count>param.recover_count+600 && !isContact[i]) // detect ground contact after 0.1 second
            {
                // leg id == i
                if(param.force_data->at(i).Fz<param.force_threshold_y)
                {
                    rt_printf("leg: %d contact ground\n",i);
                    // leg i contacted
                    isContact[i]=true;
                    contactPee[i*3+0]=pEE[0];
                    contactPee[i*3+1]=pEE[1];
                    contactPee[i*3+2]=pEE[2];
                }

            }

            if(isContact[i])
            {
                pEE[0]=contactPee[i*3+0];
                pEE[1]=contactPee[i*3+1];
                pEE[2]=contactPee[i*3+2];
                robot.SetWa(beginWa);
                robot.pLegs[i]->SetPee(pEE);
            }

        }
    }



    //for test
    if (param.count == param.align_count + param.recover_count - 1 || param.count==0)
    {
        double recoverWa;
        robot.GetWa(recoverWa);
        rt_printf("recoverWa: %f\n", recoverWa);
        double recoverPee[18];
        robot.GetPee(recoverPee);
        rt_printf("beginPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
            beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5], beginPee[6], beginPee[7], beginPee[8],
            beginPee[9], beginPee[10], beginPee[11], beginPee[12], beginPee[13], beginPee[14], beginPee[15], beginPee[16], beginPee[17]);

        rt_printf("realAlignPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
            realAlignPee[0], realAlignPee[1], realAlignPee[2], realAlignPee[3], realAlignPee[4], realAlignPee[5], realAlignPee[6], realAlignPee[7], realAlignPee[8],
            realAlignPee[9], realAlignPee[10], realAlignPee[11], realAlignPee[12], realAlignPee[13], realAlignPee[14], realAlignPee[15], realAlignPee[16], realAlignPee[17]);

        rt_printf("recoverPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
            recoverPee[0], recoverPee[1], recoverPee[2], recoverPee[3], recoverPee[4], recoverPee[5], recoverPee[6], recoverPee[7], recoverPee[8],
            recoverPee[9], recoverPee[10], recoverPee[11], recoverPee[12], recoverPee[13], recoverPee[14], recoverPee[15], recoverPee[16], recoverPee[17]);
    }



    // recover 自己做检查 //
    for (int i = 0; i < 18; ++i)
    {
        if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
        {

            if (param.motion_raw_data->at(i).target_pos > (cs.controller().motionAtAbs(i).maxPosCount() + param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is bigger than its MAX permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed %d\n",param.count);
                return 0;
            }
            if (param.motion_raw_data->at(i).target_pos < (cs.controller().motionAtAbs(i).minPosCount() - param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is smaller than its MIN permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed %d\n",param.count);
                return 0;
            }
        }
    }





    return param.align_count + param.recover_count - param.count - 1;
}
