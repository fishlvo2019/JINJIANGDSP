/*
 * AgvAppSafeManager.h
 *
 *  Created on: 2018年7月10日
 *      Author: hwei
 */

#ifndef AGVAPPSAFEMANAGER_H_
#define AGVAPPSAFEMANAGER_H_



struct AgvAppSafeManager_t
{
	int delayCount;
	int errFlag;		//置位后不再进入判断
	FH_Float power;		

};

//FH_ERROR_CODE LocationCheckOnDM(struct Location_t* location);
void AppSafeCheck();


#endif /* AGVAPPSAFEMANAGER_H_ */
