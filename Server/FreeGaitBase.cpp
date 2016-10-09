#include "FreeGaitBase.h"
#include <math.h>

namespace FreeGait
{

int Base::s_sign(double d)
{
    if(d>0)
        return 1;
    else if(d<0)
        return -1;
    else
        return 0;
}
double Base::s_variance(double*points,int n)
{
    double v=0;
    double m=s_mean(points,n);
    for(int i=0;i<n;i++)
    {
        v+=(points[i]-m)*(points[i]-m);
    }
    v=v/n;
    return v;
}

double Base::s_mean(double* points, int n)
{
    double sum=0;
    for(int i=0;i<n;i++)
    {
        sum+=points[i];
    }
    double m=sum/n;
    return m;

}
double Base::s_norm(double* vec,int n)
{
    double squareSum=0;
    for(int i=0;i<n;i++)
    {
        squareSum+=vec[i]*vec[i];
    }
    return sqrt(squareSum);
}

double Base::s_norm(double* vec1,double* vec2, int n)
{
    double squareDist=0;
    for(int i=0;i<n;i++)
    {
        squareDist+=(vec1[i]-vec2[i])*(vec1[i]-vec2[i]);
    }
    return sqrt(squareDist);
}

void Base::s_normalize(double* vec, int n)
{
    double Norm=s_norm(vec,n);
    for(int i=0;i<n;i++)
        vec[i]=vec[i]/Norm;
}

void Base::s_rx(const double rx,double* TM)
{
    TM[0]=1;
    TM[1]=0;
    TM[2]=0;
    TM[3]=0;
    TM[4]=0;
    TM[5]=cos(rx);
    TM[6]=-sin(rx);
    TM[7]=0;
    TM[8]=0;
    TM[9]=sin(rx);
    TM[10]=cos(rx);
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
}
void Base::s_ry(const double ry,double* TM)
{
    TM[0]=cos(ry);
    TM[1]=0;
    TM[2]=sin(ry);
    TM[3]=0;
    TM[4]=0;
    TM[5]=1;
    TM[6]=0;
    TM[7]=0;
    TM[8]=-sin(ry);
    TM[9]=0;
    TM[10]=cos(ry);
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
}
void Base::s_rz(const double rz,double* TM)
{
    TM[0]=cos(rz);
    TM[1]=-sin(rz);
    TM[2]=0;
    TM[3]=0;
    TM[4]=sin(rz);
    TM[5]=cos(rz);
    TM[6]=0;
    TM[7]=0;
    TM[8]=0;
    TM[9]=0;
    TM[10]=1;
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
}

void Base::s_trans(const double* trans,double*TM)
{
    TM[0]=1;
    TM[1]=0;
    TM[2]=0;
    TM[3]=trans[0];
    TM[4]=0;
    TM[5]=1;
    TM[6]=0;
    TM[7]=trans[1];
    TM[8]=0;
    TM[9]=0;
    TM[10]=1;
    TM[11]=trans[2];
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
}
//copy other matrix computation fom py


void Base::display(const double *vec,int length)
{
    if(length==16)
    {
        for(int i=0;i<4;i++)
        {
            std::cout<<vec[i*4]<<", "<<vec[i*4+1]<<", "<<vec[i*4+2]<<", "<<vec[i*4+3]<<std::endl;
        }
        std::cout<<std::endl;
    }
    else
    {
        int row;
        row=length/3;
        for(int i=0;i<row;i++)
        {
            std::cout<<vec[i*3]<<", "<<vec[i*3+1]<<", "<<vec[i*3+2]<<std::endl;
        }
        std::cout<<std::endl;
    }

}

void Base::displayRT(const double *vec,int length)
{
    if(length==16)
    {
        for(int i=0;i<4;i++)
        {
            rt_printf("%f %f %f %f\n",vec[i*4],vec[i*4+1],vec[i*4+2],vec[i*4+3]);
        }
        rt_printf("\n");
    }
    else
    {
        int row;
        row=length/3;
        for(int i=0;i<row;i++)
        {
            rt_printf("%f %f %f\n",vec[i*3],vec[i*3+1],vec[i*3+2]);
        }
        rt_printf("\n");
    }

}

void Base::s_triIncenter(const double* triCoordinates,double* center)
{
    double P1[3];
    double P2[3];
    double P3[3];
    double L1,L2,L3;

    memcpy(P1,&triCoordinates[0],sizeof(double)*3);
    memcpy(P2,&triCoordinates[3],sizeof(double)*3);
    memcpy(P3,&triCoordinates[6],sizeof(double)*3);

    L1=s_norm(P2,P3);
    L2=s_norm(P3,P1);
    L3=s_norm(P1,P2);

    center[0]=(P1[0]*L1+P2[0]*L2+P3[0]*L3)/(L1+L2+L3);
    center[1]=(P1[1]*L1+P2[1]*L2+P3[1]*L3)/(L1+L2+L3);
    center[2]=(P1[2]*L1+P2[2]*L2+P3[2]*L3)/(L1+L2+L3);
}



}
