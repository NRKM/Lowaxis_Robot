//get_counts
//6132160020-3 Yuki MITABE
//Development Environment MS Co., Ltd. Visual C++ 2013 for Windows Desktop, CONTEC Co., Ltd. CNT32-8M(PCI)

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                      Designation of Header Files                     */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

#include <conio.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <windows.h>
#include "CCNT.h"
#include "File.h"
#include "SamplingTimeForWin.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                 Declaration of Constants & Variables                 */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

//�萔
const int    LOOPMAX = 1000;      //���[�v���ő�l
const int	 dLM = LOOPMAX / 5;   //���[�v��
const double g = 9.80655;         //�d�͉����x
const double PI = 3.1415927;      //�~����
const double dt = 0.010;          //�T���v�����O�^�C��
const double Gear_Ratio = 29.0;    //�M�A��

//�ϐ�
int i = 0;
int j = 0;
int dj = 0;
float output[8] = { 0.0 };				//���͓d��
double time0 = 0.0;						//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p����ϐ�
double q[8] = { 0.0 };
double q_dot[8] = { 0.0 };
double q_ddot[8] = { 0.0 };
double qd[8] = { 0.0 };
double qd_dot[8] = { 0.0 };
double qd_ddot[8] = { 0.0 };
double q_[8] = { 0.0 };
double q_dot_[8] = { 0.0 };
double Kp[8] = { 0.0 };
double Kv[8] = { 0.0 };
double Ki[8] = { 0.0 };
double item_int = 0.0;

//�J�E���^�{�[�h
char			CNT_DevName[256] = "CNT000";
short			CNT_Id;
short			CNT_ChNo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
short			CNT_ChNum = 8;
unsigned long	CNT_data[8];

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                      Preparation of File Output                      */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

vector<double> memory_q1(LOOPMAX);
vector<double> memory_qd1(LOOPMAX);
vector<double> memory_q_dot1(LOOPMAX);
vector<double> memory_qd_dot1(LOOPMAX);
vector<double> memory_time(LOOPMAX);

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                               Function                               */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

//�J�E���^�{�[�h�I�[�v��
void CNT_open()
{
	if (CntInit(CNT_DevName, &CNT_Id) != 0)
	{
		printf("Failure CntInit=%d\n", CntInit(CNT_DevName, &CNT_Id));
	}

	for (i = 0; i<8; i++)
	{
		if (CntSelectChannelSignal(CNT_Id, CNT_ChNo[i], CNT_SIGTYPE_LINERECEIVER) != 0)
		{
			printf("Failure CntSelectChannelSignal=%d\n", CntSelectChannelSignal(CNT_Id, CNT_ChNo[i], CNT_SIGTYPE_LINERECEIVER));
		}

		if (CntSetOperationMode(CNT_Id, CNT_ChNo[i], CNT_MODE_2PHASE, CNT_MUL_X4, CNT_CLR_ASYNC) != 0)
		{
			printf("Failure CntSetOperationMode=%d\n", CntSetOperationMode(CNT_Id, CNT_ChNo[i], CNT_MODE_2PHASE, CNT_MUL_X4, CNT_CLR_ASYNC));
		}
	}

	if (CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum) != 0)
	{
		printf("Failure CntZeroClearCount=%d\n", CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if (CntStartCount(CNT_Id, CNT_ChNo, CNT_ChNum) != 0)
	{
		printf("Failure CntStartCount=%d\n", CntStartCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	return;
}

//�J�E���^�{�[�h�N���[�Y
void CNT_close()
{
	if (CntStopCount(CNT_Id, CNT_ChNo, CNT_ChNum) != 0)
	{
		printf("Failure CntStopCount=%d\n", CntStopCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if (CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum) != 0)
	{
		printf("Failure CntZeroClearCount=%d\n", CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if (CntExit(CNT_Id) != 0)
	{
		printf("Failure CntExit=%d\n", CntExit(CNT_Id));
	}

	return;
}

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                                 main                                 */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

int main()
{
	//�J�E���^�{�[�h�I�[�v��
	CNT_open();

	//���C������J�n
	getchar();
	cout << "����J�n\n";
	getchar();

	//�T���v�����O�^�C���擾�J�n
	SAMPLING_TIME sampling_timer;
	sampling_timer.Initialize(dt);

	while(1)
	{
		//�p���X�̎擾
		CntReadCount(CNT_Id, CNT_ChNo, CNT_ChNum, CNT_data);

		//�p�x,�p���x,�p�����x�̎擾
		for(i=0;i<8;i++)
		{
			q[i] = (2.0*PI) * (((double)((int)CNT_data[i])) / (4.0*512.0*Gear_Ratio));
			q_dot[i] = (q[i] - q_[i]) / dt;
			q_ddot[i] = (q_dot[i] - q_dot_[i]) / dt;
			q_[i] = q[i];
			q_dot_[i] = q_dot[i];
		}

		//�擾�l�`�F�b�N
		printf("CNT_data[]=%d\t,q[]_rad=%.2lf\t,q[]_deg=%.2lf\r", CNT_data[1], q[1], q[1] * (180 / PI));

		//�f�[�^�i�[
		if(j%5==0)
		{
			dj = j / 5;

			memory_q1[dj] = q[1];
			memory_qd1[dj] = qd[1];
			memory_q_dot1[dj] = q_dot[1];
			memory_qd_dot1[dj] = qd_dot[1];
			memory_time[dj] = time0;
		}
		sampling_timer.WaitNextSamplingTime(&time0);
		j++;

		if(GetAsyncKeyState(VK_RETURN)&&GetAsyncKeyState(VK_SHIFT))
		{
			break;
		}
	}

	//�T���v�����O�^�C���擾�I��
	sampling_timer.end();

	//�t�@�C���o��
	File File001(5.0*dt, dLM);
	File File002(5.0*dt, dLM);
	File File003(5.0*dt, dLM);

	//�f�[�^�̏o��
	File001.Fileout("time.txt", memory_time);
	File002.Fileout("q1.txt", memory_qd1, memory_q1);
	File003.Fileout("q_dot1.txt", memory_qd_dot1, memory_q_dot1);

	//�J�E���^�{�[�h�N���[�Y
	CNT_close();

	return 0;
}