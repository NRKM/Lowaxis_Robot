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
#include "Caio.h"
#include "CCNT.h"
#include "DA8ch.h"
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
const int    LOOPMAX = 5000;	//���[�v���ő�l
const int	 dLM = LOOPMAX / 5;	//���[�v��
const double g = 9.80655;		//�d�͉����x
const double PI = 3.1415927;	//�~����
const double dt = 0.011;		//�T���v�����O�^�C��
const double Kt = 0.0162;		//�g���N�萔[Nm/A]
const double Imax = 0.294;		//���[�^�ő�A���d��[A]
const double Gear_Ratio = 29.0;	//������
const double Ra = 11.2;			//�[�q�Ԓ�R[��]
const double Kr = 589;			//��]���萔[rpm/V]
const double Ke = 1 / Kr;		//�t�N�d�͒萔[V/rpm]
const double Ks = 408000;		//��]��/�g���N���z[rpm/Nm]

//�ϐ�
int i = 0;
int j = 0;
int dj = 0;
float output[8] = { 0.0 };		//���͓d��
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

//PWM�֘A
double Vm[8] = { 0.0 };				//���[�^�֏o�͓d��[V]
double N[8] = { 0.0 };				//���[�^��]���x[rpm]
double Current_Per[8] = { 0.0 };
const double dt1 = 0.0010;
double dt2 = 0.0;
double dt3 = 0.0;
double time0 = 0.0;					//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p���鎞��
double time1 = 0.0;					//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p���鎞��
double time2 = 0.0;					//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p���鎞��
double time3 = 0.0;					//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p���鎞��
double total_time = 0.0;			//���A���T���v�����O�^�C���ɂ���Ƃ��Ɏg�p���鎞�Ԃ̍��v
double on_time[8] = { 0.0 };
double off_time[8] = { 0.0 };

//DA�{�[�h
char			DA_DevName[256] = "AIO000";
short			DA_Id;
short			DA_ChNum = 8;
float			DA_data[8] = { 0.0 };

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
vector<double> memory_time1(LOOPMAX);
vector<double> memory_time2(LOOPMAX);
vector<double> memory_time3(LOOPMAX);
vector<double> memory_total_time(LOOPMAX);
vector<double> memory_Vm(LOOPMAX);
vector<double> memory_output(LOOPMAX);
vector<double> memory_ontime(LOOPMAX);
vector<double> memory_offtime(LOOPMAX);

//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*																		*/
/*                               Function                               */
/*																		*/
/************************************************************************/
//////////////////////////////////////////////////////////////////////////

//DA�{�[�h�I�[�v��
void DA_open()
{
	if (AioInit(DA_DevName, &DA_Id) != 0)
	{
		printf("Failure AioInit=%d\n", AioInit(DA_DevName, &DA_Id));
	}

	if (AioSetAoRangeAll(DA_Id, PM10) != 0)
	{
		printf("Failure AioSetAoRangeAll=%d\n", AioSetAoRangeAll(DA_Id, PM10));
	}

	return;
}

//DA�{�[�h�N���[�Y
void DA_close()
{
	if (AioMultiAoEx(DA_Id, DA_ChNum, DA_data) != 0)
	{
		printf("Failure AioMultiAoEx=%d\n", AioMultiAoEx(DA_Id, DA_ChNum, DA_data));
	}

	if (AioExit(DA_Id) != 0)
	{
		printf("Failure AioExit=%d\n", AioExit(DA_Id));
	}

	return;
}

//�J�E���^�{�[�h�I�[�v��
void CNT_open()
{
	if(CntInit(CNT_DevName, &CNT_Id)!=0)
	{
		printf("Failure CntInit=%d\n", CntInit(CNT_DevName, &CNT_Id));
	}

	for(i=0;i<8;i++)
	{
		if(CntSelectChannelSignal(CNT_Id, CNT_ChNo[i], CNT_SIGTYPE_LINERECEIVER)!=0)
		{
			printf("Failure CntSelectChannelSignal=%d\n", CntSelectChannelSignal(CNT_Id, CNT_ChNo[i], CNT_SIGTYPE_LINERECEIVER));
		}

		if(CntSetOperationMode(CNT_Id, CNT_ChNo[i], CNT_MODE_2PHASE, CNT_MUL_X4, CNT_CLR_ASYNC)!=0)
		{
			printf("Failure CntSetOperationMode=%d\n", CntSetOperationMode(CNT_Id, CNT_ChNo[i], CNT_MODE_2PHASE, CNT_MUL_X4, CNT_CLR_ASYNC));
		}
	}

	if(CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum)!=0)
	{
		printf("Failure CntZeroClearCount=%d\n", CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if(CntStartCount(CNT_Id, CNT_ChNo, CNT_ChNum)!=0)
	{
		printf("Failure CntStartCount=%d\n", CntStartCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	return;
}

//�J�E���^�{�[�h�N���[�Y
void CNT_close()
{
	if(CntStopCount(CNT_Id, CNT_ChNo, CNT_ChNum)!=0)
	{
		printf("Failure CntStopCount=%d\n", CntStopCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if(CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum)!=0)
	{
		printf("Failure CntZeroClearCount=%d\n", CntZeroClearCount(CNT_Id, CNT_ChNo, CNT_ChNum));
	}

	if(CntExit(CNT_Id)!=0)
	{
		printf("Failure CntExit=%d\n", CntExit(CNT_Id));
	}

	return;
}

//�o�̓��~�b�^
double limit_Ampere(double output)
{
	if (output >= Imax * 0.90)
	{
		output = Imax * 0.90;
	}

	if (output <= -Imax * 0.90)
	{
		output = -Imax * 0.90;
	}

	return output;
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
	//DA�{�[�h�I�[�v��
	DA_open();

	//�J�E���^�{�[�h�I�[�v��
	CNT_open();

	//���C������J�n
	getchar();
	std::cout << "����J�n\n";
	getchar();
	std::cout << "���䒆...\n";

	//�T���v�����O�^�C�}�[�֐��錾
	SAMPLING_TIME sampling_timer;

	while(1)
	{
		//�T���v�����O�^�C���������@
		sampling_timer.Initialize(dt1);

		//�ً}��~
		if(_kbhit())
		{
			for(i=0;i<DA_ChNum;i++)
			{
				output[i] = 0.0;
			}
			DAout(output);
			break;
		}

		//�p���X�̎擾
		CntReadCount(CNT_Id, CNT_ChNo, CNT_ChNum, CNT_data);

		//�ڕW�l�ݒ�
		qd[1] = PI / 2.0;

		//�p�x,�p���x,�p�����x�̎擾
		q[1] = (2.0*PI) * (((double)((int)CNT_data[1])) / (4.0*512.0*Gear_Ratio));
		q_dot[1] = (q[1] - q_[1]) / dt;
		q_ddot[1] = (q_dot[1] - q_dot_[1]) / dt;
		q_[1] = q[1];
		q_dot_[1] = q_dot[1];

		//�Q�C���ݒ�
		Kp[1] = 0.010;
		Kv[1] = 0.00255;

		//���䑦
		output[1] = Kp[1] * (qd[1] - q[1]) + Kv[1] * (qd_dot[1] - q_dot[1]);
		output[1] = output[1] / Kt;
		
		//�o�̓��~�b�^
		output[1] = float(limit_Ampere(output[1]));
		
		//�o�͓d���v�Z
		Current_Per[1] = output[1] / (Imax * 0.90);		//�d���̔䗦
		N[1] = Ks * output[1] * Kt;						//���[�^��]���x�v�Z
		Vm[1] = output[1] * Ra + Ke * N[1];				//���[�^�ւ̏o�͓d���v�Z

		//ON,OFF�^�C���v�Z
		dt2 = 0.010 * Current_Per[1];
		dt3 = 0.010 * (1.0 - Current_Per[1]);

		//�T���v�����O�^�C���ҋ@�@
		sampling_timer.WaitNextSamplingTime(&time0);
		time1 = time0;

		//ON�^�C��		
		sampling_timer.Initialize(dt2);					//�T���v�����O�^�C���������A
		output[1] = Vm[1];
		DAout(output);
		on_time[1] = output[1];
		//printf("output = %lf\n",output[1]);
		sampling_timer.WaitNextSamplingTime(&time0);	//�T���v�����O�^�C���ҋ@�A
		time2 = time0;

		//OFF�^�C��
		sampling_timer.Initialize(dt3);					//�T���v�����O�^�C���������B
		output[1] = 0.0;
		DAout(output);
		off_time[1] = output[1];
		//printf("output = %lf\n", output[1]);

		//�f�[�^�i�[
		if(j%5==0)
		{
			dj = j / 5;

			memory_q1[dj] = q[1];
			memory_qd1[dj] = qd[1];
			memory_q_dot1[dj] = q_dot[1];
			memory_qd_dot1[dj] = qd_dot[1];
			memory_time[dj] = time0;
			memory_time1[dj] = time1;
			memory_time2[dj] = time2;
			memory_time3[dj] = time3;
			memory_total_time[dj] = total_time;
			memory_Vm[dj] = Vm[1];
			memory_output[dj] = output[1];
			memory_ontime[dj] = on_time[1];
			memory_offtime[dj] = off_time[1];
		}
		sampling_timer.WaitNextSamplingTime(&time0);	//�T���v�����O�^�C���ҋ@�B
		time3 = time0;

		//�S���Ԍv�Z
		total_time = time1 + time2 + time3;
		j++;
	}

	//�o�͒�~
	for(i=0;i<DA_ChNum;i++)
	{
		output[i] = 0.0;
	}

	//DA�{�[�h�֏o��
	DAout(output);

	//�T���v�����O�^�C�}�[�I��
	sampling_timer.end();

	//�t�@�C���o��
	File File001(5.0*dt, dLM);
	File File002(5.0*dt, dLM);
	File File003(5.0*dt, dLM);
	File File004(5.0*dt, dLM);
	File File005(5.0*dt, dLM);
	File File006(5.0*dt, dLM);

	//�f�[�^�̏o��
	File001.Fileout("time.txt", memory_time, memory_total_time);
	File002.Fileout("time_wait.txt", memory_time1, memory_time2, memory_time3);
	File003.Fileout("q1.txt", memory_qd1, memory_q1);
	File004.Fileout("q_dot1.txt", memory_qd_dot1, memory_q_dot1);
	File005.Fileout("output.txt", memory_Vm, memory_output);
	File006.Fileout("time_onoff.txt", memory_ontime, memory_offtime);

	//���C������I��
	getchar();
	std::cout << "����I��\n";

	//�J�E���^�{�[�h�N���[�Y
	CNT_close();

	//DA�{�[�h�N���[�Y
	DA_close();

	return 0;
}