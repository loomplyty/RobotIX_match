#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <string>

using namespace std;

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


// TY gait
 #include "FreeGait.h"


int main(int argc, char *argv[])
{
    std::string xml_address;



    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-IX will start" << std::endl;
        xml_address = "/usr/Robots/resource/Robot_Type_III/Robot_IX/Robot_IX.xml";
    }
    else if (std::string(argv[1]) == "IX")
    {
        xml_address = "/home/hex/Desktop/codes/RobotIX_all/resource/Robot_IX.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "/home/hex/Desktop/codes/RobotIX_all/resource/Robot_VIII.xml";
    }
	else
	{
        throw std::runtime_error("invalid robot name, please type in III, VIII or IX");
	}

	auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeIII>();
	rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::Gait::basicParse, nullptr);
    rs.addCmd("ds", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hm", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hmsw", Robots::Gait::basicParse, nullptr);
    rs.addCmd("rc", Robots::Gait::recoverParse, Robots::Gait::recoverGait);
    rs.addCmd("wk", Robots::Gait::walkParse, Robots::Gait::walkGait);
    rs.addCmd("ro", Robots::Gait::resetOriginParse, Robots::Gait::resetOriginGait);
    rs.addCmd("ec", Robots::Gait::extendChainParse, Robots::Gait::extendChainGait);
    //waist cmd
    rs.addCmd("rcw", Robots::Gait::recoverWaistParse, Robots::Gait::recoverWaistGait);
    rs.addCmd("aw", Robots::Gait::adjustWaistParse, Robots::Gait::adjustWaistGait);
    rs.addCmd("rcir", Robots::Gait::recoverParse, Robots::Gait::recoverInRampGait);




    //slope walking  TY gait cmds
//    rs.addCmd("adj",VersatileGait::parseAdjustSlope,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("frc",VersatileGait::parseForce,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("imu",VersatileGait::parseIMU,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("vis",VersatileGait::parseVision,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("gsvf2",VersatileGait::parseGoSlopeVisionFast2,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("pitch",VersatileGait::parsePitch,VersatileGait::GoSlopeByVisionFast2);
//    rs.addCmd("gs35",VersatileGait::parseGoSlope35,VersatileGait::GoSlope35);



	rs.open();

	rs.setOnExit([&]()
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile(xml_address.c_str());
		auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
		if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
		rs.model().saveXml(*model_xml_ele);

		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();


	return 0;
}
