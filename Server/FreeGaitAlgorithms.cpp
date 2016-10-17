#include "FreeGaitAlgorithms.h"
namespace FreeGait{

void Algorithms::GetBodyOffset(const double pitch, const double roll, double* offset)
{
     // only for test
//    offset[0]=bodyElevation*sin(roll);
//    offset[1]=0.0;
//    offset[2]=-bodyElevation*sin(pitch);
}
void Algorithms::GetBodyOffsetRobotIX(const double pitch, const double roll, double* offset)
{
//    offset[0]=bodyElevation*sin(roll);
//    offset[1]=1.2*(bodyElevation*cos(pitch)-bodyElevation);
//    offset[2]=(-bodyElevation-0.2)*sin(pitch);// not sure depend on robot elevation
 }
void Algorithms::GetTM_C_2_B(double* Legs_2_b,double *TM_c_2_b)
{

    int firstPairID[3]={0,2,4};
    int secondPairID[3]={1,5,3};

    double firstPair[9];
    double secondPair[9];
    for (int i=0;i<3;i++)
    {
        memcpy(&firstPair[3*i],&Legs_2_b[firstPairID[i]*3],sizeof(double)*3);
        memcpy(&secondPair[3*i],&Legs_2_b[secondPairID[i]*3],sizeof(double)*3);
    }

    double y1[3];
    double y2[3];
    double y[3];
    double z[3];
    double x_prime[3]{1,0,0};
    double x[3];

    Base::s_triPlane(firstPair,y1);
    Base::s_triPlane(secondPair,y2);

    Base::s_normalize(y1);
    Base::s_normalize(y2);
    y[0]=y1[0]+y2[0];  // this is actually the same as half quaternion method
    y[1]=y1[1]+y2[1];
    y[2]=y1[2]+y2[2];
    Base::s_normalize(y);

    aris::dynamic::s_cro3(x_prime,y,z);
    aris::dynamic::s_cro3(y,z,x);
    Base::s_normalize(x);
    Base::s_normalize(z);

    double TriC_firstPair[3];
    double TriC_secondPair[3];
    Base::s_triIncenter(firstPair,TriC_firstPair);
    Base::s_triIncenter(secondPair,TriC_secondPair);

    TM_c_2_b[0]=x[0];
    TM_c_2_b[1]=y[0];
    TM_c_2_b[2]=z[0];
    TM_c_2_b[3]=0.5*TriC_firstPair[0]+0.5*TriC_secondPair[0];
    TM_c_2_b[4]=x[1];
    TM_c_2_b[5]=y[1];
    TM_c_2_b[6]=z[1];
    TM_c_2_b[7]=0.5*TriC_firstPair[1]+0.5*TriC_secondPair[1];
    TM_c_2_b[8]=x[2];
    TM_c_2_b[9]=y[2];
    TM_c_2_b[10]=z[2];
    TM_c_2_b[11]=0.5*TriC_firstPair[2]+0.5*TriC_secondPair[2];
    TM_c_2_b[15]=1;
}

}


