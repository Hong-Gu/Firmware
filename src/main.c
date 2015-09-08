 /*
  ******************************************************************************
  * @file    : main.c
  * @author  : Chang, Wei-Chieh
  * @version : V1.0.0
  * @date    : 07-September-2015
  * @brief   : This the main file of firmware of the attitude control.
  ******************************************************************************
  * @license : GNU GPL version 3 or later <http://www.gnu.org/licenses/gpl.html>.
  *
  * Copyright (C) 2015 Chang, Wei-Chieh
  *
  * This file is the main program of attitude control based on STM32F4-Discovery.
  * 
  *
  * You should have received a copy of the GNU General Public License
  * along with MAIN. If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

#include "main.h"
#include "attitude.h"
#include "controller.h"

int main()
{

	float Toservo[3] = {0};

	L3GD20_Configuration();
	

	while(1)
	{

		AttitudeControl(Toservo);

		for(int i=0; i<100000; i++);
	}

	return 0;

}
