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

/* the attitude command function */
void AttitudeControl(float* AttiCommData)
{
	/* */
	uint8_t Ki[3] = { 1, 1, 1 };
	uint8_t Ko[3] = { 1, 1, 1 };
	float RateData[3] = {0};
	float AttiData[3] = {0};
	float AttiDesr[3] = {0};
	float ErroData[3] = {0};

	/* read six degree of freedom data */
	L3GD20_ReadGyro(RateData);
	Eulerangle(AttiData);
	
	/* compute the error signal and commmand */
	for( int i=0; i<3; i++ )
	{
		ErroData[i] = AttiDesr[i] - AttiData[i];
		AttiCommData[i] = Ki[i] * ( Ko[i] * ErroData[i] - RateData[i] );
	} 	

};
