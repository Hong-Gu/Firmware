 /* @title   : controller.c
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
#include"controller.h"

/* control commmand of yaw control */
float YawControl( float InnerloopYaw, float OutterloopYaw, float Psi, float R)
{
	 return InnerloopYaw * R + OutterloopYaw * Psi;	
};

/* control command of roll control */
float RollControl(float InnerloopRoll, float OutterloopRoll, float Phi, float Q)
{
        return InnerloopRoll * Q + OutterloopRoll * Phi;
	//	RollCoom = 1.0 * Q + 1.0 * Phi;
};

/* control command of pitch control */
float PitchControl(float InnerloopPitch, float OutterloopPitch, float Theta, float P)
{
        return InnerloopPitch * P + OutterloopPitch * Theta;
	//YawCoom = 1.0 * P + 1.0 * Theta;
};

/* the attitude command function */
void AttitudeControl(float* AttiCoomData)
{
	float RateData[3] = {0};
	float AttiData[3] = {0};

	/* read six degree of freedom data */
	L3GD20_ReadGyro(RateData);
	Eulerangle(AttiData);

	/*  */
	AttiCoomData[0] = YawControl( 1.0, 1.0, AttiData[0], RateData[0] );
	AttiCoomData[1] = RollControl( 1.0, 1.0, AttiData[1], RateData[1] );
	AttiCoomData[2] = PitchControl( 1.0, 1.0, AttiData[2], RateData[2] ); 	

};
