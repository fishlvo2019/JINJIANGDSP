/*
 * AgvAppSafeManager.h
 *
 *  Created on: 2018��7��10��
 *      Author: hwei
 */

#ifndef AGVAPPSAFEMANAGER_H_
#define AGVAPPSAFEMANAGER_H_



struct AgvAppSafeManager_t
{
	int delayCount;
	int errFlag;		//��λ���ٽ����ж�
	FH_Float power;		

};

//FH_ERROR_CODE LocationCheckOnDM(struct Location_t* location);
void AppSafeCheck();


#endif /* AGVAPPSAFEMANAGER_H_ */
