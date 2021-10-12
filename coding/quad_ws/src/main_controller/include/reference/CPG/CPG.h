#ifndef _CPG_H_
#define _CPG_H_
#include "MR2Init.h"
/**
* @brief CPG计算结构体
* @param None
* @retval None
*/
typedef struct
{
	float x[4];
	float y[4];
	float Ah,Ak;
	float u;
	float cos0[4][4];
	float sin0[4][4];
	float SumX[4];
	float SumY[4];
	
}CPGCAL_s;

/**
* @brief CPG参数结构体
* @param None
* @retval None
*/
typedef struct
{
	float Delta_t;
	float Dividedtime;
	float Alpha;
	float A;
	float Ah;
	float W_sw;
	float X0;
	float Y0;
	float B;
	float Ak;
	float Theta0;
}CPGPRA_s;

extern CPGCAL_s CCPG;
extern CPGPRA_s PCPG;
void calCPG(float *ah,float *ak);
void CPGInit(void);
void updataXiYi(void);
void mat_cos_sin(void);
void CPG(void);
#endif

