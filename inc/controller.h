/*:
  ******************************************************************************
  * @file    : controller.h
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

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "main.h"

/*
#define InnerloopPitch 1;
#define InnerloopRoll  1; 
#define InnerloopYaw  1;  


#define OutterloopPitch  1;
#define OutterloopRoll  1;
#define OutterloopYaw  1;
*/


/* the function of controller */
float YawControl( float, float, float, float );
float RollControl( float, float, float, float );
float PitchControl( float, float, float, float );
void AttitudeControl( float*);

#endif

