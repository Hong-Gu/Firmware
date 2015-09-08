  /*
  ******************************************************************************
  * @file    : attitude.h
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

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "main.h"

void L3GD20_Configuration(void);
void L3GD20_ReadGyro(float*);
void Eulerangle(float*);

#endif