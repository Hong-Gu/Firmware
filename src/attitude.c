/*:
  ******************************************************************************
  * @file    : attitude.c
  * @author  : Chang, Wei-Chieh
  * @version : V1.0.0
  * @date    : 07-September-2015
  * @brief   : 
  ******************************************************************************
  * @license : GNU GPL version 3 or later <http://www.gnu.org/licenses/gpl.html>.
  *
  * Copyright (C) 2015 Chang, Wei-Chieh
  *
  *
  * You should have received a copy of the GNU General Public License
  * along with MAIN. If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

#include"attitude.h"

void L3GD20_Configuration(void)
{
	/* L3GD20 Configuration */
  	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	
	/* Configure for L3GD20 */
  	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
 	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
 	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250; 
  	L3GD20_Init(&L3GD20_InitStructure);
  
  	/* Configure High-Pass Filter for L3GD20 */
  	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
 	L3GD20_FilterConfig(&L3GD20_FilterStructure);

  	/* Configure High-Pass Filter for L3GD20 */ 
  	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
};


void L3GD20_ReadGyro(float* pfData)
{
  	uint8_t tmpbuffer[6] = {1.0};
  	int16_t RawData[3] = {0};
  	uint8_t tmpreg = 0;
  
  	/* read data from specific register */
  	L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  	L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);


  	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  	if(!(tmpreg & 0x40))
  	{
    		for(int i=0; i<3; i++)
    		{
      			RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    		}
  	}
  	else
  	{
    		for(int i=0; i<3; i++)
    		{
      			RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    		}
  	}	

  	/* divide by sensitivity in 250dps */
  	for( int i=0; i<3; i++)
  	{
    		pfData[i] = (float)RawData[i] / 114.285f;
  	}	
};

void Eulerangle(float* AttiData)
{
  	float GyroData[3] = {0};
  	float dt = 0.01f; 
  	float E[4] = {0};
  	float dE[4] = {0};
	
	/* read the anguler velocity */
  	L3GD20_Configuration();
	L3GD20_ReadGyro(GyroData);  

  	/* state euqation of Quaternions */
  	dE[0] = 0.5 * ( -GyroData[0] * E[1] - GyroData[1] * E[2] - GyroData[2] * E[3] );
  	dE[1] = 0.5 * (  GyroData[0] * E[0] + GyroData[2] * E[2] - GyroData[1] * E[3] );
  	dE[2] = 0.5 * (  GyroData[1] * E[0] - GyroData[2] * E[1] + GyroData[0] * E[3] );
  	dE[3] = 0.5 * (  GyroData[2] * E[0] + GyroData[1] * E[1] - GyroData[0] * E[2] );

  	/* the euler integration */
  	for(int i = 0; i<3; i++)
  	{
    		E[i] = E[i] + dE[i] * dt;
  	}

  	/* converge the Quaternions to Euler angle */
  	AttiData[0] = atan( 2*( E[0]*E[1] + E[2]*E[3] ) / \
			( pow( E[0], 2 ) - pow( E[1], 2 ) - pow(E[2],2) + pow(E[3],2) ) ); 
  	AttiData[1] = atan( 2*( E[0]*E[3] + E[1]*E[2] ) / \
			( pow( E[0], 2 ) + pow( E[1], 2 ) - pow(E[2],2) - pow(E[3],2) ) );
  	AttiData[2] = asin( 2*( E[0]*E[2] - E[3]*E[1] ) );

};

