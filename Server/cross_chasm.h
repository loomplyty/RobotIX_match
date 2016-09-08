/* ��̬���ܣ��繵
* by liujimu, 2016-7-7
*/

/*������ע�ʹ�����ӵ�xml�ļ�*/
/*
            <cc default="cc_param">
                <cc_param type="group">
                    <totalCount abbreviation="t" type="int" default="3000"/>
                    <chasmWidth abbreviation="d" type="double" default="1"/>
                    <stepHeight abbreviation="h" type="double" default="0.1"/>
					<dir_param type="unique" default="forward">
						<forward abbreviation="f"/>
						<backward abbreviation="b"/>
					</dir_param>
				</cc_param>
            </cc>
*/

#ifndef CROSS_CHASM_H
#define CROSS_CHASM_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Type_III.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct ccParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 3000 };
	double chasmWidth{ 1 };
    double stepHeight{ 0.1 };
	bool isBackward{ false };
};

auto crossChasmParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto crossChasmGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

struct wvParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
	int legSequence{ 0 };//���ȴ���0��1��2��-1��ʾ������
    double stepHeight{ 0.1 };
	double stepLength{ 0 };
	double stepWidth{ 0 };
	double targetPeb[6]{ 0 };
};

auto waveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CROSS_CHASM_H
