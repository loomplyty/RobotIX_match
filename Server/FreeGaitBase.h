#ifndef FREEGAITBASE_H
#define FREEGAITBASE_H
//#include <thread>
//#include <functional>
//#include <cstdint>
//#include <map>
#include <cstring>
#include <iostream>
#include "rtdk.h"
#include "aris_dynamic.h"

namespace FreeGait
{
namespace Base
{
//*** basic mathematics functions***//
int s_sign(double d);
double s_variance(double *points,int n);
double s_mean(double *points, int n);
double s_norm(double *vec, int n=3);
double s_norm(double *vec1,double* vec2, int n=3);
void s_normalize(double *vec, int n=3);

//***matrix computation***//
void s_rx(const double rx,double *TM);
void s_ry(const double ry,double *TM);
void s_rz(const double rz,double *TM);
void s_trans(const double *trans,double *TM);
//copy other matrix computation fom py

//***Display***//
void displayRT(const double *vec,int length);
void display(const double *vec,int length);

//**geometry**//
void s_triIncenter(const double *triCoordinates, double *center);
void s_triPlane(const double *triCoordinates, double *normalVec);

}}
#endif
