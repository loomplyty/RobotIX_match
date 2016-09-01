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

// CZJ gait 
#include "Move_Gait.h"

// TY gait
#include "Vision_Gait0.h"
#include "Kinect2.h"
#include "Kinect2Test.h"
#include "GaitGenerator.h"

Kinect2Sensor::KINECT2 kinect2;

static auto visionSlopeThread = std::thread([]()
{
    sleep(1);
    cout << "Sleep for a second waiting for something to init\n";

    while(true)
    {
        VersatileGait::ScanningInfo info;
        VersatileGait::visionSlopePipe.recvInNrt(info);
        // cout<<"planner demanding received!"<<endl;

        if(info.isInit == true)
        {
            //kinect2.InitMap();
            if(VersatileGait::FlagV==VersatileGait::FlagVision::Free)
            {
                VersatileGait::FlagV=VersatileGait::FlagVision::VisionScanning;
                memcpy(VersatileGait::gridMapBuff,kinect2.visData.gridMap,sizeof(float)*400*400);
                // rt_printf("map buffer is (in vision thread): %f %f\n",VersatileGait::gridMapBuff[200][200],VersatileGait::gridMapBuff[300][200]);
                VersatileGait::FlagV=VersatileGait::FlagVision::Free;
            }

        }
        else
        {
            float TM_float[16];
            for(int i=0;i<16;i++)
                TM_float[i]=float(info.TM[i]);
           // kinect2.GetPose(TM_float);
            cout<<"Transformation Matrix got in Vision!"<<endl;
            for(int i=0;i<4;i++)
            {
                cout<<TM_float[i*4]<<" "<<TM_float[i*4+1]<<" "<<TM_float[i*4+2]<<" "<<TM_float[i*4+3]<<" "<<endl;
            }
           // kinect2.UpdateConMap();

            if(VersatileGait::FlagV==VersatileGait::FlagVision::Free)
            {
                VersatileGait::FlagV=VersatileGait::FlagVision::VisionScanning;
                memcpy(VersatileGait::gridMapBuff,kinect2.visData.gridMap,sizeof(float)*400*400);
                // rt_printf("map buffer is (in vision thread): %f %f\n",VersatileGait::gridMapBuff[200][200],VersatileGait::gridMapBuff[300][200]);
                VersatileGait::FlagV=VersatileGait::FlagVision::Free;
            }
            //cout<<"map update"<<endl;
        }

        //  cout<<" map recorded to shared memory"<<endl;
        VersatileGait::isScanningFinished = true;
    }
});


int main(int argc, char *argv[])
{
    ForceTask::ForceWalk forcewalker;
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


    rs.addCmd("cmb",ForceTask::parseContinueMoveBegin,ForceTask::continueMove);
    rs.addCmd("cmj",ForceTask::parseContinueMoveJudge,ForceTask::continueMove);
    rs.addCmd("odb",ForceTask::parseOpenDoorBegin,ForceTask::openDoor);
    rs.addCmd("odj",ForceTask::parseOpenDoorJudge,ForceTask::openDoor);
    rs.addCmd("ffd",Robots::walkParse, ForceTask::forceForward);

    // CZJ gait cmd
    rs.addCmd("fcw",forcewalker.parseForceWalk,forcewalker.forceWalk);

    //slope walking  TY gait cmds
    rs.addCmd("adj",VersatileGait::parseAdjustSlope,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("frc",VersatileGait::parseForce,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("imu",VersatileGait::parseIMU,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("vis",VersatileGait::parseVision,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("gsvf2",VersatileGait::parseGoSlopeVisionFast2,VersatileGait::GoSlopeByVisionFast2);

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
