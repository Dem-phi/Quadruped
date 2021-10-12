#include "CPG.h"
#include <math.h>

CPGPRA_s PCPG;
CPGCAL_s CCPG;
float PSI[4] = {0,0.5,0.5,0}; //tort 
//float PSI[4] = {0,0.5,0.25,0.75}; //wall
void updataXiYi(){
    float upSumX[4] ={ 0},upSumY[4] = {0};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            upSumX[i] +=  CCPG.cos0[i][j] * CCPG.x[j] - CCPG.sin0[i][j] * CCPG.y[j];
            upSumY[i] +=  CCPG.cos0[i][j] * CCPG.y[j] + CCPG.sin0[i][j] * CCPG.x[j];
        }

    for(int i=0;i<4;i++)
    {
        CCPG.SumX[i] = upSumX[i];
        CCPG.SumY[i] = upSumY[i];
    }

}
void CPG(){
    updataXiYi();
    for(int i=0 ; i<4 ;i++){
        float w = (1-PCPG.B) * PCPG.W_sw /PCPG.B/(exp(-PCPG.A*CCPG.y[i])+1) + PCPG.W_sw/(exp(PCPG.A * CCPG.y[i])+1);
        float r2 = pow(CCPG.x[i],2)+pow(CCPG.y[i],2);
        float delta_x = PCPG.Alpha * (CCPG.u-  r2) * CCPG.x[i] - w*CCPG.y[i] + CCPG.SumX[i];
        float delta_y = PCPG.Alpha * (CCPG.u-  r2) * CCPG.y[i] + w*CCPG.x[i] + CCPG.SumY[i];
        CCPG.x[i] += delta_x * PCPG.Delta_t;
        CCPG.y[i] += delta_y * PCPG.Delta_t;
    }

}


void calCPG(float *ah,float *ak){
    CPG();
    for(int i=0;i<4;i++){
        ah[i] = CCPG.x[i];
        ak[i] = 1 * CCPG.Ak  * CCPG.y[i] / CCPG.Ah;
    }
}

/**
* @brief CPG关系矩阵参数初始化
* @param None
* @retval None
* @TODO None
*/
void mat_cos_sin(){
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            CCPG.cos0[i][j] = 	cos(2*PI*(PSI[i] - PSI[j]));
            CCPG.sin0[i][j] = 	sin( 2*PI*(PSI[i] - PSI[j]));
        }
}


void CPGInit(){
    PCPG.A = 50;
    PCPG.Ah = 9.8;
    PCPG.Ak = 8.3;
    PCPG.Alpha = 100;
    PCPG.B = 0.5;
    //PCPG.B = 0.75;

    PCPG.W_sw = 4 * PI;
    PCPG.X0 = 1;
    PCPG.Y0 = 1;
    PCPG.Theta0 = PI / 6;

    PCPG.Delta_t = 0.001;
    PCPG.Dividedtime = 1;
    mat_cos_sin();
    for(int i=0;i<4;i++)
    {
        CCPG.x[i] =	PCPG.X0;
        CCPG.y[i] =	PCPG.Y0;
    }
    CCPG.Ak = PCPG.Ak * D2R;
    CCPG.Ah = PCPG.Ah * D2R;//asin(B*V*T/(4*L*cos(THETA0)));
    CCPG.u= pow(CCPG.Ah,2);
}

