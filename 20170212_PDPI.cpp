#ifdef _DEBUG
//Debug���[�h�̏ꍇ
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#else
//Release���[�h�̏ꍇ
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#endif

#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <windows.h>
#include <conio.h>

//AIO�w�b�_�t�@�C��
#include "caio.h"
#include "CCnt.h"

#include <cv.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#pragma comment(lib, "winmm.lib")

void CALLBACK TimerProc(UINT wID,UINT uMsg,DWORD dwUser,DWORD dw1,DWORD dw2);
void control();

#include <mmsystem.h>//���荞�ݏ���

#define TIMER_EVENT   1		//1000���̉��b��
#define M_PI	3.1415926535

//���荞�ݏ���
TIMECAPS tc;
MMRESULT tid;

double GetSysTime(void);		//���Ԃ��擾����֐�

#define FRAME_WIDTH		320		// �摜�̃T�C�Y
#define FRAME_HEIGHT	240
#define ROI_WIDTH01		/*170*/250		// �����_01�̘g�T�C�Y
#define ROI_HEIGHT01	240
#define ROI_WIDTH02		/*270*/290		// �����_02�̘g�T�C�Y
#define ROI_HEIGHT02	230

FILE *fp0;
FILE *fp1;

// ����o��
cv::VideoWriter video01;
cv::VideoWriter video02;

const double f	= (1000 / cv::getTickFrequency());
int64 nowTime	= 0;   // ������
int64 pastTime	= 0;
int64 diffTime	= 0;   // �o�ߎ���
int64 startTime = 0;

// DA�{�[�h�Z�b�e�B���O
char	DA_DevName[256] = "AIO002";
short	DA_Id;
short	DA_ChNum = 16;
float	DA_data[16] = {0.0};

// CNT�{�[�h�Z�b�e�B���O
char	CNT_DevName[256] = "CNT000";
short	CNT_Id;
short	CNT_ChNo[8]={0,1,2,3,4,5,6,7};
short	CNT_ChNum = 8;
unsigned long	CNT_data[8];

//�p�x�E�p���x
double q[6]		= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//�p�x[rad](�����p�x)
double q_pr[6]	= { 0.0 };		//1���[�v�O�̊p�x[rad]
double q_dot[6] = { 0.0 };		//�p���x[rad/s]

double fq[6]	 = { 0.0 };
double fq_dot[6] = { 0.0 };

double fq_pr[6]	 = { 0.0 };
double fq_dot_pr[6] = { 0.0 };

//���ʒu�E�ڕW�ʒu
//double z_position = 0.0;
//double zd		  = 0.0;

double x;
double y;

//****************************************pololu motor********************************************//
double qq[8] = { 0.0 };
double qq_[8] = {0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0};
double qq_dot[8] = { 0.0 };
double qq_dot_low[8] = {0.0};				//���[�p�X��̌��݃��[�^�p���x
double qq_dot_low_before[8] = {0.0};			//���[�p�X�O

double qq_pr[8]	= { 0.0 };		//1���[�v�O�̊p�x[rad]
double fqq[8]	 = { 0.0 };
double fqq_pr[8]	 = { 0.0 };
double sum_qq[8] = { 0.0 };
//************************************************************************************************//

//������͗p//���䑥
double qd[6] = {0.0};       //�ڕW�p�x[rad]
double qd_dot[6] = {0.0};      //�ڕW�p���x
double sum[6] = { 0.0 };
double sum_q[6] = { 0.0 };
double sum_diff[6] = {0.0};
double M[4]  = { 0.22, 0.41, 0.41, /*1.63*/3.20 };	//�����N�d��
double L[3]  = { 0.15, 0.30, 0.17 };				//�����N����
double G = 9.81;

//�o�l�ɂ��d�͕⏞�@�\
double k[2] = {800, 750};
double d[2] = {0.13, 0.20};
double r[2] = {0.03, 0.04};
double Z[4] = { 0.0 };

//double sqrt (double A);
double A = 0.0;
double C = 0.0;
double B = 0.0;
double D = 0.0;

// Gain Setting
double KPe[6] = {0.0};      //�p�x�t�B�[�h�o�b�N�Q�C��(�ʒu�t�B�[�h�o�b�N)
double KVe[6] = {0.0};      //�p���x�t�B�[�h�o�b�N�Q�C��(���x�t�B�[�h�o�b�N)
double KIe[6] = {0.0};      //�ϕ��t�B�[�h�o�b�N�Q�C��

double KPc[6] = {0.0};      //���o�t�B�[�h�o�b�N�Q�C��
double KIc[6] = {0.0};      //�ϕ��t�B�[�h�o�b�N�Q�C��

double a[4] = { 0.0 };      //g[i]���ғ�
double Q[4] = { 0.0 };
double g[4] = { 0.0 };     //�d�͕⏞��

double check = {0.0};

double sum_perx =0;
double sum_pery =0;
double sumx =0;
double sumy =0;

int loop = 0;
int loop_tmp = 0;
int counter = 1;
int p = 0;

//******************************pololu motor***********************************//
double cntmax3 = 8995358.4673298448;
double Gear_Ratio_3 = 1000.0;					//�쓮���[�^�M�A��
double kt3 = 0.551730;						    //�g���N�萔[Nm/A]
//*****************************************************************************//

//���R�r
double J[24] = {0.0};
double Jx[4] = {0.0};
double Jx_pr[4] = {0.0};

//���[�p�X�t�B���^
double LOWPASS(double src, double prev);
double dst_prev = 0.0;

double LOWPASS2(double src2, double prev2);
double dst_prev2 = 0.0;

cv::Point mkrPos[3][3], tarPos[3][3], target[4][3], tarPos1[3][3], target1[3][3];

//�J�����̐؂�ւ��^�C�~���O
double temp_mkrPos[3][3][2] = {0.0};
double per_temp_mkrPos[3][3][2] = {0.0};

double low_mkrPos[3][3][2] = {0.0};
double low_mkrPos_pr[3][3][2] = {0.0};

//�J�����X�V�ɂ�郍�{�b�g�̓�������
double mrk_cam_x= 0.0;
double mrk_cam_y= 0.0;
double mrk_cam_z= 0.0;
double pre_mrk_cam_x= 0.0;
double pre_mrk_cam_y= 0.0;
double pre_mrk_cam_z= 0.0;

// �ڕW�Ǝ��̕΍����i�}�[�J���j
CvPoint3D32f diff1, diff2, diff3;
CvPoint3D32f diff1_pr;
CvPoint3D32f sum_diff1, sum_diff2, sum_diff3;

// �J�����t���O
int flg_cam[10][2] = {0};	// �J�����t���O�i���R�Ɏg����j
int cam_alive[2] = {4,4};	// �J�����������Ă��邩�H
int flg_record = 0;		// �����t�@�C���ɏ����o�����ǂ����H
int endsys = 0;
int tarFlg = 999;
int tarFlg2 = 999;
int init_qFlg = 0;
int handFlg = 0;

int AN_loop = 0;

//�J����
int i=0;
int j=0;

//clock_t start,end;
clock_t start0,start1,end = 0.0;

void main()
{
	//�t�@�C���o��
	fopen_s(&fp0, "20170320_tau_001.csv", "w");
	fopen_s(&fp1, "20170320_delta_x_001.csv", "w");

	fprintf_s(fp0, "�摜����,Time[s],M0KPe,M1KPe,M2KPe,M3KPe,M0KVe,M1KVe,M2KVe,M3KVe,M0KIe,M1KIe,M2KIe,M3KIe,M0KPc,M1KPc,M2KPc,M3KPc,M0KIc,M1KIc,M2KIc,M3KIc,��0,��1,��2,��3,��4\n");
	fprintf_s(fp1, "�摜����,Time[s],��0,��1,��2,��3,��4,��5,��6,��7,��8,q0,q1,q2,q3,q4,q5,q6,qd0,qd1,qd2,qd3,qd4,qd5,qd6,q_dot0,q_dot1,q_dot2,q_dot3,q_dot4,q_dot5,qd_dot0,qd_dot1,qd_dot2,qd_dot3,qd_dot4,qd_dot5,qd_dot��x[pic],��y[pic],��z[pic],��x[mm],��y[mm],��z[mm],��x,g0,g1\n");

	//DA������
	AioInit( DA_DevName, &DA_Id );
	AioSetAoRangeAll( DA_Id , PM10 );

	//CNT������
	CntInit ( CNT_DevName , &CNT_Id );
	CntZeroClearCount ( CNT_Id , CNT_ChNo , CNT_ChNum );
	CntStartCount ( CNT_Id , CNT_ChNo , CNT_ChNum );

	check = 0;

	///�{�[�h�̏���&�f�[�^�����@�����܂�
	////////////////////////////////////

	//�^�C�}�[�J�n//���荞��//
	timeGetDevCaps(&tc,sizeof(tc));
	timeBeginPeriod(tc.wPeriodMin);
	tid=timeSetEvent(TIMER_EVENT,0,TimerProc,0,TIME_PERIODIC);

	char text[255] = "";

	cv::VideoCapture cap_1(0);
	cap_1.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap_1.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	cv::VideoCapture cap_2(1);
	cap_2.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap_2.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

	cv::Mat src_img[3];
	cv::Mat show_img[3];
	cap_1 >> src_img[0];
	cap_2 >> src_img[1];

	cv::namedWindow("VISION-1", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("VISION-2", CV_WINDOW_AUTOSIZE);

	cv::Point ROI[1][3] = { {cv::Point(0, 0),cv::Point(30, 0),cv::Point(30, 10)} };	// �ڕW�t�߂̒����_�i�g�̍���j

	//�摜�o��
	video01.open("20170320_video01_001.avi", CV_FOURCC('X','V','I','D'), 30, cv::Size(FRAME_WIDTH,FRAME_HEIGHT), 1);
	video02.open("20170320_video02_001.avi", CV_FOURCC('X','V','I','D'), 30, cv::Size(FRAME_WIDTH,FRAME_HEIGHT), 1);

	for( i=0; i<3; i++){
		for(j=0; j<3; j++){
			mkrPos[j][i].x = 0;	
			mkrPos[j][i].y = 0;	
			tarPos[j][i].x = 0;	
			tarPos[j][i].y = 0; 
			target[j][i].x = 0; 
			target[j][i].y = 0;
			tarPos1[j][i].x = 0;
			tarPos1[j][i].y = 0;
			target1[j][i].x = 0;
			target1[j][i].y = 0;
		}
	}

	int tarHSV[4] = {75, 90, 50, 50};	// �ڕW�̐F���iHSV�j255�X�P�[��
	int tarHSV1[4] = {20, 45, 50, 45};
	int tarHSV2[4] = /*{170, 180, 50, 50}*/{0, 10, 50, 50};
	int mkrHSV[4] = /*{0, 10, 50, 50}*//*{90, 110, 70, 70}*/{0, 10, 50, 50};

	cv::Mat result_img = cv::Mat(cv::Size(FRAME_WIDTH,FRAME_HEIGHT),CV_8UC3);
	cv::Mat result_img1 = cv::Mat(cv::Size(FRAME_WIDTH,FRAME_HEIGHT),CV_8UC3);

	cv::Mat smooth_img, smooth_img1/*, smooth_img2*/;
	cv::Mat hsv_img, hsv_img1/*, hsv_img2*/;

	cv::Point sum, sum1/*, sum2*/;
	cv::Point mkrPoints[40000];
	cv::Point tarPoints[40000];
	cv::Point tarPoints1[40000];

	int camNum = 0;
	int roiNum = 0;
	int mkrCount = 0;
	int tarCount = 0;
	int tarCount1 = 0;
	int x = ROI[5][5].x;
	int y = ROI[5][5].y;
	int a;

	startTime = cv::getTickCount();

	while(1)
	{
		sum_perx = sumx;
		sum_pery = sumy;
		result_img = cv::Scalar(0,0,0);
		result_img1 = cv::Scalar(0,0,0);
		cap_1 >> src_img[0];
		cap_2 >> src_img[1];


		//�摜�擾���Ԍv��
		pastTime = diffTime;
		nowTime = cv::getTickCount();
		diffTime = (int)((nowTime- startTime)*f);
		std::ostringstream os;
		os << diffTime;
		std::string number = os.str();

		/*Vision01�����J�n*/
		cv::medianBlur(src_img[0],smooth_img,7);	//�m�C�Y������̂ŕ�����
		cv::cvtColor(smooth_img,hsv_img,CV_BGR2HSV);	//HSV�ɕϊ�
		for(i=0; i<40000; i++){
			mkrPoints[i].x = 0;
			mkrPoints[i].y = 0;	
		}
		for( i=0; i<40000; i++){	
			tarPoints[i].x = 0;	
			tarPoints[i].y = 0;	
		}

		for(int y=ROI[0][0].y; y<ROI[0][0].y + ROI_HEIGHT01; y++){
			for(int x=ROI[0][0].x; x<ROI[0][0].x + ROI_WIDTH01; x++){
				int a = hsv_img.step*y+(x*3);
				if( hsv_img.data[a] >= mkrHSV[0] && hsv_img.data[a] <= mkrHSV[1] && hsv_img.data[a+1] >= mkrHSV[2] && hsv_img.data[a+2] >= mkrHSV[3] ){
					result_img.data[a]   = 255;
					result_img.data[a+1] = 255;
					result_img.data[a+2] = 255;
				}
				if( hsv_img.data[a]   >= tarHSV[0] && hsv_img.data[a]   <= tarHSV[1] &&	hsv_img.data[a+1] >= tarHSV[2] && hsv_img.data[a+2] >= tarHSV[3] ){
						result_img.data[a]   = 100;
						result_img.data[a+1] = 255;
						result_img.data[a+2] = 255;
				}
			}
		}
		cv::erode(result_img, hsv_img, cv::Mat(), cv::Point(-1,-1), 1);
		cv::erode(hsv_img, result_img, cv::Mat(), cv::Point(-1,-1), 1);

		//�摜�����͈͂����߂�
		mkrCount = 0;
		tarCount = 0;
		tarCount1 = 0;
		for(int y=ROI[0][0].y; y<ROI[0][0].y + ROI_HEIGHT01;y++){
			for(int x=ROI[0][0].x; x<ROI[0][0].x + ROI_WIDTH01; x++){
				a = hsv_img.step*y+(x*3);
				if( hsv_img.data[a] == 255 ){
					mkrPoints[mkrCount].x = x;
					mkrPoints[mkrCount].y = y;
					mkrCount++;
				} else if( hsv_img.data[a] == 100 ){
					tarPoints[tarCount].x = x;
					tarPoints[tarCount].y = y;
					tarCount++;
				}
			}
		}
		// �F�̈�̏d�S���Z�o����
		if(mkrCount>0){
			sum.x = 0; sum.y = 0;
			for(int i = 0; i < mkrCount; ++i){
				sum.x += mkrPoints[i].x;
				sum.y += mkrPoints[i].y;

				sumx=sum.x;
				sumy=sum.y;
			}

			per_temp_mkrPos[0][0][0] = temp_mkrPos[0][0][0];
			per_temp_mkrPos[0][0][1] = temp_mkrPos[0][0][1];
			mkrPos[0][0].x = (int)(sum.x / mkrCount);
			mkrPos[0][0].y = (int)(sum.y / mkrCount);
			temp_mkrPos[0][0][0] = ((double)sum.x/(double)mkrCount);
			temp_mkrPos[0][0][1] = ((double)sum.y/(double)mkrCount);
		}
		if(tarCount>20){
			sum.x = 0; sum.y = 0;
			for( i = 0; i < tarCount; ++i){
				sum.x += tarPoints[i].x;
				sum.y += tarPoints[i].y;
			}
			tarPos[0][0].x = (int)(sum.x / tarCount);
			tarPos[0][0].y = (int)(sum.y / tarCount);
		}/*Vision01�����I��*/

		//window�ɓ_�⎞�Ԃ��o��
		cv::rectangle( src_img[0], cv::Point(ROI[0][0].x, ROI[0][0].y), cv::Point(ROI[0][0].x+ROI_WIDTH01, ROI[0][0].y+ROI_HEIGHT01), cv::Scalar(255,0,0), 3, 8 );//ROI
		cv::circle( src_img[0], mkrPos[0][0], 4, CV_RGB(0,255,0), -1, 8, 0 );
		cv::circle( src_img[0], target[0][0], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
        cv::circle( src_img[0], target[1][0], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
        cv::circle( src_img[0], target[2][0], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
		cv::circle( src_img[0], target[3][0], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
		cv::putText( src_img[0], number, cv::Point(5,20), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,200), 1, CV_AA );

		/*Vision02�����J�n*/
		cv::medianBlur(src_img[1],smooth_img1,7);	//�m�C�Y������̂ŕ�����
		cv::cvtColor(smooth_img1,hsv_img1,CV_BGR2HSV);	//HSV�ɕϊ�
		for(i=0; i<40000; i++){
			mkrPoints[i].x = 0;
			mkrPoints[i].y = 0;	
		}
		for( i=0; i<40000; i++){	
			tarPoints[i].x = 0;	
			tarPoints[i].y = 0;	
		}

		for(int y=ROI[0][1].y; y<ROI[0][1].y + ROI_HEIGHT02; y++){
			for(int x=ROI[0][1].x; x<ROI[0][1].x + ROI_WIDTH02; x++){
				int a = hsv_img1.step*y+(x*3);
				if( hsv_img1.data[a] >= mkrHSV[0] && hsv_img1.data[a] <= mkrHSV[1] && hsv_img1.data[a+1] >= mkrHSV[2] && hsv_img1.data[a+2] >= mkrHSV[3] ){
					result_img1.data[a]   = 255;
					result_img1.data[a+1] = 255;
					result_img1.data[a+2] = 255;
				}
				if( 
					hsv_img1.data[a]   >= tarHSV[0] && hsv_img1.data[a]   <= tarHSV[1] &&	hsv_img1.data[a+1] >= tarHSV[2] && hsv_img1.data[a+2] >= tarHSV[3] ){
						result_img1.data[a]   = 100;
						result_img1.data[a+1] = 255;
						result_img1.data[a+2] = 255;
				}
			}
		}
		cv::erode(result_img1, hsv_img1, cv::Mat(), cv::Point(-1,-1), 1);
		cv::erode(hsv_img1, result_img1, cv::Mat(), cv::Point(-1,-1), 1);

		//�摜�����͈͂����߂�
		mkrCount = 0;
		tarCount = 0;
		for(int y=ROI[0][1].y; y<ROI[0][1].y + ROI_HEIGHT02;y++){
			for(int x=ROI[0][1].x; x<ROI[0][1].x + ROI_WIDTH02; x++){
				a = hsv_img1.step*y+(x*3);
				if( hsv_img1.data[a] == 255 ){
					mkrPoints[mkrCount].x = x;
					mkrPoints[mkrCount].y = y;
					mkrCount++;
				} else if( hsv_img1.data[a] == 100 ){
					tarPoints[tarCount].x = x;
					tarPoints[tarCount].y = y;
					tarCount++;
				}
			}
		}
		// �F�̈�̏d�S���Z�o����
		if(mkrCount>0){
			sum1.x = 0; sum1.y = 0;
			for(int i = 0; i < mkrCount; ++i){
				sum1.x += mkrPoints[i].x;
				sum1.y += mkrPoints[i].y;

				sumx=sum1.x;
				sumy=sum1.y;
			}

			per_temp_mkrPos[0][1][0] = temp_mkrPos[0][1][0];
			per_temp_mkrPos[0][1][1] = temp_mkrPos[0][1][1];
			mkrPos[0][1].x = (int)(sum1.x / mkrCount);
			mkrPos[0][1].y = (int)(sum1.y / mkrCount);
			temp_mkrPos[0][1][0] = ((double)sum1.x/(double)mkrCount);
			temp_mkrPos[0][1][1] = ((double)sum1.y/(double)mkrCount);
		}
		if(tarCount>20){
			sum1.x = 0; sum1.y = 0;
			for( i = 0; i < tarCount; ++i){
				sum1.x += tarPoints[i].x;
				sum1.y += tarPoints[i].y;
			}
			tarPos[0][1].x = (int)(sum1.x / tarCount);
			tarPos[0][1].y = (int)(sum1.y / tarCount);
		}
		/*Vision02�����I��*/

		//window�ɓ_�⎞�Ԃ��o��
		cv::rectangle( src_img[1], cv::Point(ROI[0][1].x, ROI[0][1].y), cv::Point(ROI[0][1].x+ROI_WIDTH02, ROI[0][1].y+ROI_HEIGHT02), cv::Scalar(255,0,0), 3, 8 );//ROI
		cv::circle( src_img[1], mkrPos[0][1], 4, CV_RGB(0,255,0), -1, 8, 0 );
		cv::circle( src_img[1], target[0][1], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
		cv::circle( src_img[1], target[1][1], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
		cv::circle( src_img[1], target[2][1], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
        cv::circle( src_img[1], target[3][1], /*15*/10, CV_RGB(0,0,255), 2, 8, 0 );
		cv::putText( src_img[1], number, cv::Point(5,20), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,200), 1, CV_AA );

		cv::imshow("VISION-1",src_img[0]);
		cv::imshow("VISION-2",src_img[1]);

		// �摜�o��
		cv::Mat output_img01=src_img[0];
		video01 << output_img01;

		cv::Mat output_img02=src_img[1];
		video02 << output_img02;


		int key = cvWaitKey( 1 );
		if(key==27){ break; }

		switch( key ){
			//�S����~
			case 's':
				tarFlg=6;

				for(int i=0; i<DA_ChNum; i++){
					DA_data[i] = 0.0;
				}
				AioMultiAoEx(DA_Id, DA_ChNum, DA_data);
				break;

			//�֐ߊp�x����p����PD����(�O������) + ���o����
			case 'z':
				sum_q[0] = 0.0;		sum_q[1] = 0.0;		sum_q[2] = 0.0;		sum_q[3] = 0.0;
				startTime = cv::getTickCount();
				loop = 0;

				tarFlg=0;
				break;

			//�����ʒu�֖߂�
			case 'o':
                loop = 0;

				tarFlg=100;
				break;

	    	//���o����p����PD-PI����
			/*case 'x':
				sum_diff[0] = 0.0;		sum_diff[1] = 0.0;		sum_diff[2] = 0.0;
				tarFlg=1;

				check = 0;
				break;*/

				//�ڕW�l�`�F�b�N
			case 't':     //1�_��
				/*tarFlg=7;*/
				tarHSV[0] = 75;
				tarHSV[1] = 90;
				tarHSV[2] = 50;
				tarHSV[3] = 50;

                        target[0][0].x = tarPos[0][0].x + /*20.0*/90.0;
						target[0][0].y = tarPos[0][0].y;
						target[0][1].x = tarPos[0][1].x;
						target[0][1].y = tarPos[0][1].y;
				break;

            case 'y':
                tarFlg/*2*/=7;
                break;

			case 'g':     //2�_��
				/*tarFlg=8;*/
				tarHSV[0] = /*20*/75;
				tarHSV[1] = /*45*/90;
				tarHSV[2] = /*50*//*55*/50;
				tarHSV[3] = /*45*/50;

                        target[1][0].x = tarPos[0][0].x /*+*/ /*20.0*//*60.0*/;
						target[1][0].y = tarPos[0][0].y + 70.0;
						target[1][1].x = tarPos[0][1].x /*- 3.0*/;
						target[1][1].y = tarPos[0][1].y + 50.0;
				break;

            case 'h':
                tarFlg/*2*/=8;
				AN_loop=0;
                break;

			case 'b':     //3�_��
				/*tarFlg=9;*/
				tarHSV[0] = /*90*/0;
				tarHSV[1] = /*110*/10;
				tarHSV[2] = /*65*//*70*/50;
				tarHSV[3] = /*70*/50;

                        target[2][0].x = tarPos[0][0].x + /*20.0*//*40.0*/83.0;
						target[2][0].y = tarPos[0][0].y;
						target[2][1].x = tarPos[0][1].x + 15.0;
						target[2][1].y = tarPos[0][1].y;
				break;

            case 'n':
                tarFlg/*2*/=9;
                break;

			case 'r':     //4�_��
				/*tarFlg=9;*/
				tarHSV[0] = 0;
				tarHSV[1] = 10;
				tarHSV[2] = 50;
				tarHSV[3] = 50;

                        target[3][0].x = tarPos[0][0].x + /*20.0*//*40.0*/80.0;
						target[3][0].y = tarPos[0][0].y /*- 35.0*/-30.0;
						target[3][1].x = tarPos[0][1].x + 60.0/*-50.0*/;
						target[3][1].y = tarPos[0][1].y /*- 35.0*/-30.0;
				break;

            case 'f':
                tarFlg/*2*/=10;
                break;

			//�O���b�p�̊J��
			case 'u':     //��
				tarFlg=70;
				AN_loop=0;
				break;

			case 'i':     //�J
				tarFlg=77;
				AN_loop=0;
				break;

			case 'j':     //Motor3����
				tarFlg=78;
				AN_loop=0;
				break;

			case 'k':     //Motor123�J
				tarFlg=79;
				AN_loop=0;
				break;

			default:
				break;
		}

	}

	for(int i=0; i<DA_ChNum; i++){
		DA_data[i] = 0.0;
	}

	// DA�I������
	AioMultiAoEx(DA_Id, DA_ChNum, DA_data);
	AioExit( DA_Id );

	CntStopCount( CNT_Id , CNT_ChNo , CNT_ChNum );
	CntZeroClearCount( CNT_Id , CNT_ChNo , CNT_ChNum );
	CntExit(CNT_Id);

	fclose(fp0);
	fclose(fp1);
	//fclose(fp3);

}

void control(){

	if(tarFlg==0){	//z
		if(start0 == 0.0){
			start0 = clock();
		}

		if(loop <= 3000){
			qd[0] =  /*30.0*/33.0 / 180.0 * M_PI * (-2.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)+3.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)) +  /*30.0*/20.0 / 180.0 * M_PI;
			qd[1] =	/*-30.0*/-18.0 / 180.0 * M_PI * (-2.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)+3.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)) + 170.0 / 180.0 * M_PI;
			qd_dot[0] =  /*30.0*/33.0 / 180.0 * M_PI * (-6.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)*(1.0/3.0)+6.0*((double)loop*0.001/3.0)*(1.0/3.0));
			qd_dot[1] = /*-30.0*/-18.0 / 180.0 * M_PI * (-6.0*((double)loop*0.001/3.0)*((double)loop*0.001/3.0)*(1.0/3.0)+6.0*((double)loop*0.001/3.0)*(1.0/3.0));

		KPe[0] = /*25.0*/27.1;	KPe[1] = /*25.0*/26.5;	KPe[2] = /*0.5*/0.6;	KPe[3] = 4.0;
		KVe[0] = /*12.6*/13.0;	KVe[1] = /*12.6*/12.8;	KVe[2] = 0.6;	KVe[3] = 1.2;
		KIe[0] = 0.0;   KIe[1] = 0.0;   KIe[2] = 0.0;   KIe[3] = 0.0;

		KPc[0] = 0.0;	KPc[1] = 0.0;	KPc[2] = 0.0;	KPc[3] = 0.0;
		KIc[0] = 0.0;	KIc[1] = 0.0;	KIc[2] = 0.0;	KIc[3] = 0.0;

		a[0] = 0.8;     a[1] = /*0.4*/0.85;     a[2] = 0.0;     a[3] = 0.0;
		Q[0] = 1.0;		Q[1] = 1.0;

		}else if(loop > 3000/* && loop <= 6000*/){
			/*qd[0] =  60.0 / 180.0 * M_PI;
			qd[1] = 150.0 / 180.0 * M_PI;
			qd[3] =  90.0 / 180.0 * M_PI;
			qd_dot[0] = 0.0;
			qd_dot[1] = 0.0;
			qd_dot[3] = 0.0;

		KPe[0] = 12.0;	KPe[1] = 12.0;	KPe[2] = 0.5;	KPe[3] = 4.0;
		KVe[0] = 6.0;	KVe[1] = 6.0;	KVe[2] = 0.5;	KVe[3] = 1.2;
		KIe[0] = 1.8;   KIe[1] = 1.8;   KIe[2] = 0.0;   KIe[3] = 0.1;

		KPc[0] = 0.0;	KPc[1] = 0.0;	KPc[2] = 0.0;	KPc[3] = 0.0;
		KIc[0] = 0.0;	KIc[1] = 0.0;	KIc[2] = 0.0;	KIc[3] = 0.0;

		a[0] = 0.4;     a[1] = 0.40.8;     a[2] = 0.0;     a[3] = 0.0;
		Q[0] = 1.0;		Q[1] = 1.0;*/

		//}/*else{
			sum_diff[0] = 0.0;		sum_diff[1] = 0.0;		sum_diff[2] = 0.0;
			tarFlg=1;
			printf("x\n");
		}

		qd[2] = /*15.0*/19.5 * 2.0 * M_PI;		//���ׂ�l�W�ɂ����i(�ڕW�ړ�����[mm]*2.0��)(1��]��2[mm])

		loop_tmp = 0;
		//motion_check8= 1;

	}else if(tarFlg==100){  //o

		if(loop < 10000){
			qd[0] = -15.0 / 180.0 * M_PI * (-2.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)+3.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)) + 45.0 / 180.0 * M_PI;
			qd[1] =	 15.0 / 180.0 * M_PI * (-2.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)+3.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)) + 165.0 / 180.0 * M_PI;
			qd_dot[0] = -15.0 / 180.0 * M_PI * (-6.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)*(1.0/10.0)+6.0*((double)loop*0.001/10.0)*(1.0/10.0));
			qd_dot[1] =  15.0 / 180.0 * M_PI * (-6.0*((double)loop*0.001/10.0)*((double)loop*0.001/10.0)*(1.0/10.0)+6.0*((double)loop*0.001/10.0)*(1.0/10.0));
		}else{
			qd[0] =  30.0 / 180.0 * M_PI;
			qd[1] = 180.0 / 180.0 * M_PI;
			qd_dot[0] = 0.0;
			qd_dot[1] = 0.0;
		}
		qd[2] =	0.0 * 2.0 * M_PI;		//���ׂ�l�W�ɂ����i(�ڕW�ړ�����[mm]*2.0��)

		KPe[0] = 7.5;	KPe[1] = 7.5;	KPe[2] = 0.5;	KPe[3] = 5.0;
		KVe[0] = 3.0;	KVe[1] = 3.0;	KVe[2] = 0.5;	KVe[3] = 1.0;
		KIe[0] = 0.7;   KIe[1] = 0.7;   KIe[2] = 0.0;   KIe[3] = 0.0;

		KPc[0] = 0.0;	KPc[1] = 0.0;	KPc[2] = 0.0;	KPc[3] = 0.0;
		KIc[0] = 0.0;	KIc[1] = 0.0;	KIc[2] = 0.0;	KIc[3] = 0.0;

		a[0] = 0.4;     a[1] = 0.4;     a[2] = 0.0;     a[3] = 0.0;

		loop_tmp = 0;
		//motion_check8= 1;

	}else if(tarFlg==1){	//x
        KPe[0] = /*0.00*/0.01;	KPe[1] = /*0.00*/0.01;	KPe[2] = 0.01;	KPe[3] = 4.0;
		KVe[0] = /*6.0*/6.3;	KVe[1] = 6.0;	KVe[2] = 0.001;	KVe[3] = 1.5;
		KIe[0] = 0.0;   KIe[1] = 0.0;   KIe[2] = 0.0;   KIe[3] = 0.0;

		KPc[0] = /*0.081*/0.075;	KPc[1] = 0.081;	KPc[2] = 2.200;	KPc[3] = 0.000;
		KIc[0] = 0.045;	KIc[1] = 0.045;	KIc[2] = 1.200;	KIc[3] = 0.000;

		a[0] = 0.5;     a[1] = 0.5;     a[2] = 0.0;     a[3] = 0.0;

		//motion_check8=2;

		if(counter == 1){
			loop_tmp = loop;
			counter = 0;
		}
	}else if(tarFlg==7){ //��
		printf("Target Set\r");
	}

	//CNT�o��
	CntReadCount( CNT_Id , CNT_ChNo , CNT_ChNum , CNT_data );

	for(int i=0; i<6; i++){
		q_pr[i] = q[i];
		q[i] = (double)CNT_data[i];
		if(q[i] > 210000000.0){
			q[i] = q[i] - 4294967295.0;
		}

		// ��̒l/����\(�M�A)/������
		//*****************************�p�����������N*********************************//
		if(i == 0)	q[0] = /*-1.0 */ ( q[0] / 1024.0 / 318.0 * 2.0 * M_PI) + M_PI / /*6.0*/9.0;
		if(i == 1)	q[1] =  -1.0 * ( q[1] / 1024.0 / 318.0 * 2.0 * M_PI) + (170.0/180.0)*M_PI;
		if(i == 2)	q[2] = q[2] /  512.0 /  19.0 * 2.0 * M_PI;
		if(i == 3)	q[3] = q[3] /  512.0 / 370.0 * 2.0 * M_PI;

		//*******************************�������@�\***********************************//
		if(i == 4)	q[4] = q[4] /  512.0 /  19.0 * 2.0 * M_PI;
		if(i == 5)	q[5] = q[5] /  512.0 /  29.0 * 2.0 * M_PI;

		q_dot[i] = (q[i] - q_pr[i]) / 0.001;	//�J�E���^�p���x

		//**********************************Pololu Motor******************************************// 
		qq[6] = -(2.0*M_PI) / (12.0 * 0.25 * Gear_Ratio_3) * CNT_data[6];         //���݊p�x�Z�o
		if(qq[6] < -cntmax3/2.0){
			qq[6] = qq[6] + cntmax3;
		}
		qq_dot[6] = (qq[6] - qq_[6]) / 0.001;

		//lowpass �ǉ�
		fqq_pr[6]	 = fqq[6];
		fqq[6]		 = LOWPASS(qq[6], fqq_pr[6]);
		qq_dot_low[6] = LOWPASS(qq_dot[6], qq_dot_low_before[6]);
		qq_dot_low_before[6] = qq_dot_low[6];

		qq_[6] = qq[6];
		//*****************************************************************************************// 
		
		//���[�p�X�t�B���^
		fq_pr[i]	 = fq[i];
		fq_dot_pr[i] = fq_dot[i];
		
		fq[i]		 = LOWPASS(q[i], fq_pr[i]);
		fq_dot[i]	 = LOWPASS(q_dot[i], fq_dot_pr[i]);

	}

	    qd[3] =	M_PI - fq[1];
	    qd_dot[3] = -fq_dot[1];

		//�΍��ϕ��v�Z
		for(int i=0; i<6; i++){
		sum_q[i] += (qd[i] - fq[i])*0.001;
	}

		for(int i=6; i<7; i++){
		sum_qq[i] += (qd[i] - fqq[i])*0.001;
	}

	//�d�͕⏞
    A = 0.03*0.03+0.13*0.13-2.0*0.03*0.13*cos(90.0/180.0*M_PI - (fq[0]));
    C = sqrt(A);
    B = 0.04*0.04+0.20*0.20-2.0*0.20*0.04*cos(90.0/180.0*M_PI - (fq[1]));
    D = sqrt(B);
	g[0] = ((M[1]*G*(L[1]/2.0)*cos(fq[0]) + M[2]*G*(L[1]/2.0)*cos(fq[0]) + M[3]*G*L[1]*cos(fq[0])) / (37.842*0.6))*10.0;
    g[1] = ((M[0]*G*(L[0]/2.0)*cos(fq[1]) + M[2]*G*L[0]*cos(fq[1]) + M[3]*G*((1.0*(L[0]+L[2]))/2.0)*cos(fq[1])) / (37.842*0.6))*10.0;
	g[2] = 0.0;
	g[3] = 0.0;
	Z[0] = - ((k[0]*d[0]*r[0]*(1.0 - (0.08/C))*cos(fq[0])) / (37.842*0.6)*10.0);
	Z[1] = - ((k[1]*d[1]*r[1]*(1.0 - (0.115/D))*cos(fq[1])) / (37.842*0.6)*10.0);

	//�΍��ۑ�
	diff1_pr.x = diff1.x;
	diff1_pr.y = diff1.y;
	diff1_pr.z = diff1.z;

	//�΍��v�Z
	if(tarFlg/*2*/==8){
	diff1.x = (float)( 1.0 * ((double)target[1][1].x - temp_mkrPos[0][1][0]));
	diff1.y = (float)( -1.0 * ((double)target[1][0].y - temp_mkrPos[0][0][1]));
	diff1.z = (float)( -1.0 * ((double)target[1][0].x - temp_mkrPos[0][0][0]));
	}
	else if(tarFlg/*2*/==9 || tarFlg==70 || /*tarFlg==78 ||*/ tarFlg==79){
    diff1.x = (float)( 1.0 * ((double)target[2][1].x - temp_mkrPos[0][1][0]));
	diff1.y = (float)( -1.0 * ((double)target[2][0].y - temp_mkrPos[0][0][1]));
	diff1.z = (float)( -1.0 * ((double)target[2][0].x - temp_mkrPos[0][0][0]));
	}
	else if(tarFlg==10){
    diff1.x = (float)( 1.0 * ((double)target[3][1].x - temp_mkrPos[0][1][0]));
	diff1.y = (float)( -1.0 * ((double)target[3][0].y - temp_mkrPos[0][0][1]));
	diff1.z = (float)( -1.0 * ((double)target[3][0].x - temp_mkrPos[0][0][0]));
	}
	else{
	diff1.x = (float)( 1.0 * ((double)target[0][1].x - temp_mkrPos[0][1][0]));
	diff1.y = (float)( -1.0 * ((double)target[0][0].y - temp_mkrPos[0][0][1]));
	diff1.z = (float)( -1.0 * ((double)target[0][0].x - temp_mkrPos[0][0][0]));
	}

	per_temp_mkrPos[0][1][0] = temp_mkrPos[0][1][0];
	per_temp_mkrPos[0][0][1] = temp_mkrPos[0][0][1];
	per_temp_mkrPos[1][0][0] = temp_mkrPos[1][0][0];

	pre_mrk_cam_x = mrk_cam_x;
	pre_mrk_cam_y = mrk_cam_y;
	pre_mrk_cam_z = mrk_cam_z;

		//���R�r�s��
		J[0] = -1.0*L[1]*sin(q[0]);
		J[1] = -1.0*(L[0]+L[2])*sin(q[1]);
		J[2] = 0.0;
		J[3] = 0.0;
		J[4] = L[1]*cos(q[0]);
		J[5] = (L[0]+L[2])*cos(q[1]);
		J[6] = 0.0;
		J[7] = 0.0;
		J[8] = 0.0;
		J[9] = 0.0;
		J[10] = 1.0/1000.0*M_PI;
		J[11] = 0.0;
		J[12] = 0.0;
		J[13] = 0.0;
		J[14] = 0.0;
		J[15] = 0.0;
		J[16] = 0.0;
		J[17] = 0.0;
		J[18] = 0.0;
		J[19] = 0.0;
		J[20] = 1.0;
		J[21] = 1.0;
		J[22] = 0.0;
		J[23] = 1.0;

	for(int i=0; i<4; i++){
		Jx_pr[i] = Jx[i];
	}

	Jx[0] = (J[0]*diff1.x + J[4]*diff1.y + J[8]*diff1.z);
	Jx[1] = (J[1]*diff1.x + J[5]*diff1.y + J[9]*diff1.z);
	Jx[2] = (J[2]*diff1.x + J[6]*diff1.y + J[10]*diff1.z);
	Jx[3] = (J[3]*diff1.x + J[7]*diff1.y + J[11]*diff1.z);

	if(Jx_pr[0]/Jx[0] < 0.0)	sum_diff[0] = 0.0;
	if(Jx_pr[1]/Jx[1] < 0.0)	sum_diff[1] = 0.0;
	if(Jx_pr[2]/Jx[2] < 0.0)	sum_diff[2] = 0.0;
	if(Jx_pr[3]/Jx[3] < 0.0)	sum_diff[3] = 0.0;

	sum_diff[0] = sum_diff[0] + (J[0]*diff1.x + J[4]*diff1.y + J[8]*diff1.z)*0.001;
	sum_diff[1] = sum_diff[1] + (J[1]*diff1.x + J[5]*diff1.y + J[9]*diff1.z)*0.001;
	sum_diff[2] = sum_diff[2] + (J[2]*diff1.x + J[6]*diff1.y + J[10]*diff1.z)*0.001;
	sum_diff[3] = sum_diff[3] + (J[3]*diff1.x + J[7]*diff1.y + J[11]*diff1.z)*0.001;

	printf("Time:%lf,	ST:%lld,	x:%lf,	y:%lf,	z:%lf\r",(double)(end-start0)/CLOCKS_PER_SEC,diffTime-pastTime,diff1.x*1.218,diff1.y*0.843,diff1.z*0.843);

	//���䎮
	//*********************************************************************************�p�����������N*************************************************************************************************************//
	DA_data[0] = (float)( KPe[0] * ( qd[0] - fq[0] ) + KVe[0] * (qd_dot[0] - fq_dot[0]) + KIe[0] * sum_q[0] + KPc[0]*(J[0]*diff1.x + J[4]*diff1.y + J[8]*diff1.z)  + KIc[0]*sum_diff[0] + a[0]*g[0] + Q[0]*Z[0]);
	DA_data[1] = (float)-1.0*( KPe[1] * ( qd[1] - fq[1] ) + KVe[1] * (qd_dot[1] - fq_dot[1]) + KIe[1] * sum_q[1] + KPc[1]*(J[1]*diff1.x + J[5]*diff1.y + J[9]*diff1.z)  + KIc[1]*sum_diff[1] + a[1]*g[1] + Q[1]*Z[1]);
	DA_data[2] = (float)( KPe[2] * ( qd[2] - fq[2] ) - KVe[2] *  fq_dot[2] + KIe[2] * sum_q[2] + KPc[2]*(J[2]*diff1.x + J[6]*diff1.y + J[10]*diff1.z) + KIc[2]*sum_diff[2] + a[2]*g[2]);
	DA_data[3] = (float)( KPe[3] * ( qd[3] - fq[3] ) + KVe[3] * (qd_dot[3] - fq_dot[3]) + KIe[3] * sum_q[3] + KPc[3]*(J[3]*diff1.x + J[7]*diff1.y + J[11]*diff1.z) + KIc[3]*sum_diff[3] + a[3]*g[3]);
	
	//*****************************************�������@�\********************************************//
	//DA_data[4] = (float)( KPe[4] * ( qd[4] - fq[4] ) - KVe[4] *  fq_dot[4] + KIe[4] * sum_q[4] );
	//DA_data[5] = (float)( KPe[5] * ( qd[5] - fq[5] ) - KVe[5] *  fq_dot[5] ); //+ KIe[5] * sum_q[5] );

	//***********************************************Pololu motor**************************************************************//
	//DA_data[6] = (float)( KPe[6] * ( qd[6] - fqq[6] ) - KVe[6] * qq_dot_low[6] + KIe[6] * sum_qq[6] ) / (kt3*Gear_Ratio_3);

	for(int i=0; i<4; i++){
		if(DA_data[i] > 10.0){
			DA_data[i] = 10.0;
		}else if(DA_data[i] < -10.0){
			DA_data[i] = -10.0;
		}
	}

		/*for(int i=4; i<6; i++){
		if(DA_data[i] > 2.8){
			DA_data[i] = 2.8;
		}else if(DA_data[i] < -2.8){
			DA_data[i] = -2.8;
		}
	}*/

	/*for(int i=6; i<7; i++){
		if(DA_data[i] > 9.2){
			DA_data[i] = 9.2;
		}else if(DA_data[i] < -7.0){
			DA_data[i] = -7.0;
		}
	}*/

	if(fabs(KPc[1]) > 0.0000 && fabs(KIc[1]) > 0.0000){
		if(fabs(diff1.z*0.843) < /*50*/8.0){
			DA_data[2] = 0.0;
		}

	//�΂��物�ւ̈ړ����ɃO���b�p�J
	//if(tarFlg/*2*/==8){
	//	if(AN_loop < 60){
 //           DA_data[4] = /*4.0*/0.0;
	//		printf("v\n");
	//		}else if(AN_loop > 60){
	//		DA_data[4] = 0.0;
	//	}
	//}

	//������ւ̈ړ���ɃO���b�p��
	/*if(tarFlg==9){
		if(fabs(diff1.x*1.218) < 10.0 && fabs(diff1.y*0.843) < 10.0 && fabs(diff1.z*0.843) < 45.0){
			DA_data[0] = 0.4*g[0];
			DA_data[1] = -0.4*g[0];
			DA_data[5] = 4.0;
			printf("q\n");
		}
	}*/

	//2�s�N�Z���ȓ��ŏo�͒�~
		if(fabs(diff1.x*1.218) < /*12.0*/8.0 && fabs(diff1.y*0.843) < /*12.0*/8.0 && fabs(diff1.z*0.843) < 8.0){
			//motion_check8 = 10;
			printf("\n����");
				DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
				DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		}
	}

//****************************************�������@�\�ғ�*************************************//
	if(tarFlg==70){			//u
		//***************�p�����������N***************//
			DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
			DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
			DA_data[2] = 0.0;	DA_data[3] = 0.0;
		//***********************************************************************************//

		if(AN_loop < 300){		//�O���b�p�T�E�U��

  //      //***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***********************************************************************************//
			//KPe[4] = 0.0;   KVe[4] = 0.0;					//�e�[�v�����㉺�@�\
			//KPe[5] = 0.0;   KVe[5] = 0.0;					//�O���b�p�U�㉺
			DA_data[4] = 0.0;	DA_data[5] = 0.0;

			DA_data[6] = 0.0;

			DA_data[7] = 2.8;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;								//�O���b�p�U�J��

		}
		
		/*else */if(AN_loop >= 300 && AN_loop <= 5300){			//�O���b�p�U�������ֈړ�(���ׂ�l�W)
			if(q[5]/M_PI <= 55.0){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***********************************************************************************//
			//KPe[4] = 0.0;   KVe[4] = 0.0;					//�e�[�v�����㉺�@�\
			DA_data[4] = 0.0;
			KPe[5] = 1.0;   KVe[5] = 0.9;					//�O���b�p�U�㉺

			DA_data[6] = 0.0;

			DA_data[7] = 2.7;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 0.0;								//�O���b�p�U�J��

			qd[5] = 29.0 * 2.0 * M_PI;

			}
			else if(q[5]/M_PI > 55.0){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////*************************************************************************************//
			KPe[4] = 0.0;   KVe[4] = 0.0;					//�e�[�v�����㉺�@�\
			KPe[5] = 1.0;   KVe[5] = 0.9;					//�O���b�p�U�㉺
			//DA_data[4] = 0.0;	DA_data[5] = 0.0;

			DA_data[6] = 0.0;

			DA_data[7] = 2.8;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;								//�O���b�p�U�J��

			qd[5] = 29.0 * 2.0 * M_PI;

			//}

			}
			
		}/*else if(AN_loop > 5300){
			tarFlg=80;
		}*/

	//}
		//else if(tarFlg==80){			//�e�[�v�������O�։ғ�
		if(AN_loop > 5300 && AN_loop <= 10300){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////************************************************************************************//
			KPe[4] = 0.6;   KVe[4] = 1.0;	KIe[4] = 0.04;	//�e�[�v�����㉺�@�\
			//loop_tmp = 0;
			KPe[5] = 0.0;   KVe[5] = 0.0;					//�O���b�p�U�㉺
			//DA_data[5] = 0.0;

			DA_data[6] = 0.0;								//�e�[�v������]

			DA_data[7] = 2.8;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;								//�O���b�p�U�J��

			qd[4] = 35.0 * 2.0 * M_PI;

		}
		/*else */if(AN_loop > 10300 && AN_loop <= 16300){		//�e�[�v������]

			if(qq[6] > -1.0 * M_PI*1.35){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***********************************************************************************//
			KPe[4] = 0.0;   KVe[4] = 0.0;	KIe[4] = 0.0;	//�e�[�v�����㉺�@�\
			KPe[5] = 0.0;   KVe[5] = 0.0;					//�O���b�p�U�㉺
			//DA_data[4] = 0.0;	DA_data[5] = 0.0;

			KPe[6] = 0.0/*335.0*/;   KVe[6] = /*45.0*/0.0;	KIe[6] = /*115.0*/0.0;		//�e�[�v������]

			DA_data[7] = 2.8;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;								//�O���b�p�U�J��

			//qd[4] = 35.0 * 2.0 * M_PI;

			qd[6] = - 1.0 * M_PI*1.45;

			}

			else if(qq[6] <=  -1.0*M_PI*1.39 && qq[6] > -1.0*M_PI*1.45){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***********************************************************************************//
			//KPe[4] = 0.0;   KVe[4] = 0.0;	KIe[4] = 0.0;	//�e�[�v�����㉺�@�\
			//KPe[5] = 0.0;   KVe[5] = 0.0;					//�O���b�p�U�㉺
			DA_data[4] = 0.0;	DA_data[5] = 0.0;

				//KPe[6] = 0.0;   KVe[6] = 0.0;	KIe[6] = 0.0;		//�e�[�v������]
			DA_data[6] = 0.0;

			DA_data[7] = 2.8;								//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;								//�O���b�p�U�J��

			//qd[4] = 35.0 * 2.0 * M_PI;

			qd[6] = - 1.0 * M_PI*1.45;

			}

		}/*else if(AN_loop > 16300){
			tarFlg=81;
		}*/
		//}

		//else if(tarFlg==81){										//�e�[�v������]�����ʒu
		if(AN_loop > 16300 && AN_loop <= 24300){

			 if(qq[6] < -1.0*M_PI*1.35){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***************************************************************************************//
			//KPe[4] = 0.0;   KVe[4] = 0.0;	KIe[4] = 0.0;		//�e�[�v�����㉺�@�\
			//KPe[5] = 0.0;   KVe[5] = 0.0;						//�O���b�p�U�㉺
			DA_data[4] = 0.0;	DA_data[5] = 0.0;

			KPe[6] = /*300.0*/0.0;   KVe[6] = /*45.0*/0.0;	KIe[6] = /*90.0*/0.0;		//�e�[�v������]

			DA_data[7] = 2.8;							//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8; 							//�O���b�p�U�J��

			 qd[6] = 0.0 * M_PI;

			 }
			 else if(qq[6] < 0.0 && qq[6] >= - 1.0 * 0.2){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////***************************************************************************************//
			DA_data[4] = 0.0;	DA_data[5] = 0.0;

				KPe[6] = 0.0;   KVe[6] = 0.0;	KIe[6] = 0.0;		//�e�[�v������]
				//DA_data[6] = 0.0;

			DA_data[7] = 2.8;							//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8; 							//�O���b�p�U�J��

			qd[6] = 0.0 * M_PI;

			 }
			 //qd[6] = 0.0 * M_PI;
			 //loop_tmp = 0;
		}
		/*else */
		if(AN_loop > 24300 && AN_loop <= 29300){				//�e�[�v��������։ғ�

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////****************************************************************************************//
			KPe[4] = /*0.85*/0.8;   KVe[4] = 1.0;	KIe[4] = 0.1;			//�e�[�v�����O��@�\
			//KPe[5] = 0.0;   KVe[5] = 0.0;							//�O���b�p�U�㉺
			DA_data[5] = 0.0;

			//KPe[6] = 0.0;   KVe[6] = 0.0;	KIe[6] = 0.0;			//�e�[�v������]
			DA_data[6] = 0.0;

			DA_data[7] = 2.8;										//�O���b�p�T(�㕔)�J��
			DA_data[8] = 2.8;										//�O���b�p�U�J��

			qd[4] = 0.0 * 2.0 * M_PI;

		}
		/*else if(AN_loop > 29300){
			tarFlg=90;
		}*/


		//}

		//else if(tarFlg==90){								//�O���b�p�T�U�J��
		 if(AN_loop > 29300 && AN_loop <= 29400){

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////*****************************************************************************************//
			KPe[4] = 0.0;   KVe[4] = 0.0;	KIe[4] = 0.0;		//�e�[�v�����㉺�@�\
			KPe[5] = 0.0;   KVe[5] = 0.0;						//�O���b�p�U�㉺
			//DA_data[4] = 0.0;	DA_data[5] = 0.0;

			DA_data[6] = 0.0;									//�e�[�v������]

			DA_data[7] = -0.6;									//�O���b�p�T(�㕔)�J��
			DA_data[8] = -0.6;									//�O���b�p�U�J��

		  //loop_tmp = 0;
		 }
		 /*else */if(AN_loop > 29400 /*&& AN_loop <= 23650*/){		//�O���b�p�U���㕔�ֈړ�(���ׂ�l�W)

		////***************�p�����������N***************//
		//	DA_data[0] = ( 0.8*g[0] + 1.0*Z[0] );
		//	DA_data[1] = -1.0*( 0.85*g[1] + 1.0*Z[1] );
		//	DA_data[2] = 0.0;	DA_data[3] = 0.0;
		////*****************************************************************************************//
			//KPe[4] = 0.0;   KVe[4] = 0.0;	KIe[4] = 0.0;		//�e�[�v�����㉺�@�\
			DA_data[4] = 0.0;
			KPe[5] = 1.5;   KVe[5] = 1.0;						//�O���b�p�U�㉺

			DA_data[6] = 0.0;									//�e�[�v������]

			DA_data[7] = 0.0;									//�O���b�p�T(�㕔)�J��
			DA_data[8] = 0.0;									//�O���b�p�U�J��

			KPe[2] = 0.6;   KVe[2] = 0.6;						//�O���b�p�U�㉺

			qd[2] = 0.0 * 2.0 * M_PI;
			qd[5] = 0.0 * 2.0 * M_PI;


		 }
		 //qd[5] = 0.0 * 2.0 * M_PI;

		//}
	DA_data[2] = (float)( KPe[2] * ( qd[2] - fq[2] ) - KVe[2] *  fq_dot[2] );
//*****************************************�������@�\********************************************//
	DA_data[4] = (float)( KPe[4] * ( qd[4] - fq[4] ) - KVe[4] *  fq_dot[4] + KIe[4] * sum_q[4] );
	DA_data[5] = (float)( KPe[5] * ( qd[5] - fq[5] ) - KVe[5] *  fq_dot[5] ); //+ KIe[5] * sum_q[5] );

//***********************************************Pololu motor**************************************************************//
	DA_data[6] = (float)( KPe[6] * ( qd[6] - fqq[6] ) - KVe[6] * qq_dot_low[6] + KIe[6] * sum_qq[6] ) / (kt3*Gear_Ratio_3);

	for(int i=2; i<3; i++){
		if(DA_data[i] > 10.0){
			DA_data[i] = 10.0;
		}else if(DA_data[i] < -10.0){
			DA_data[i] = -10.0;
		}
	}

	for(int i=4; i<6; i++){
		if(DA_data[i] > 2.8){
			DA_data[i] = 2.8;
		}else if(DA_data[i] < -2.8){
			DA_data[i] = -2.8;
		}
	}

	for(int i=6; i<7; i++){
		if(DA_data[i] > 9.2){
			DA_data[i] = 9.2;
		}else if(DA_data[i] < -7.0){
			DA_data[i] = -7.0;
		}
	}


		}

//*************************************************************************************************************************//


	//	if(tarFlg==78){  //j
	//	if(AN_loop < 90){
	//		DA_data[0] = 0.4*g[0];
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = /*-6.0*/-3.5;
	//		DA_data[5] = -3.5;
	//		DA_data[6] = 6.5;
	//	}else if(AN_loop > 100 && AN_loop < /*190*//*350*/350){
 //           DA_data[0] = 0.4*g[0];
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = /*-6.0*//*-3.5*/0.0;
	//		DA_data[5] = -3.0;
	//		DA_data[6] = 6.0;
	//	}
	//	else{
 //           DA_data[0] = 0.4*g[0];
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = 0.0/*-3.5*//*-6.0*/;
	//		DA_data[5] = -3.5;
	//		DA_data[6] = 0.0;
	//	}
	//}


	//if(tarFlg==79){  //k
	//	if(AN_loop < /*100*/80){
	//		DA_data[0] = 0.4*g[0];;
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = 0.0;
	//		DA_data[5] = /*0.0*/6.0;
	//		DA_data[6] = /*-7.0*/0.0;
	//	}
	//	else if(AN_loop > /*160*/500 && AN_loop < /*200*/600){
	//		DA_data[0] = 0.4*g[0];;
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = 0.0;
	//		DA_data[5] = 0.0;
	//		DA_data[6] = -7.0;
	//	}
	//	else if(AN_loop > /*160*/2500 && AN_loop < /*200*/2580){
 //           DA_data[0] = 0.4*g[0];
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = 5.0;
	//		DA_data[5] = /*6.0*/0.0;
	//		DA_data[6] = 0.0;
	//	}
	//	else{
	//		DA_data[0] = 0.4*g[0];
	//		DA_data[1] = -0.4*g[1];
	//		DA_data[2] = 0.0;
	//		DA_data[3] = 0.0;
	//		DA_data[4] = 0.0;
	//		DA_data[5] = 0.0;
	//		DA_data[6] = 0.0;
	//	}
	//}

		//if(tarFlg==70){  //u
		//	if(AN_loop < /*20*/40){
		//		DA_data[0] = 0.4*g[0];
		//		DA_data[1] = -0.4*g[1];
		//		DA_data[4] = /*6.0*/-3.5;
		//		DA_data[5] = -3.5;
		//		printf("u\n");
		//	}else if(AN_loop > /*20*/40){
		//		DA_data[0] = 0.4*g[0];
		//		DA_data[1] = -0.4*g[1];
		//		DA_data[2] = 0.0;
		//		DA_data[3] = 0.0;
		//		DA_data[4] = /*3.0*/-3.5;
		//		DA_data[5] = /*-3.5*/-3.5;

		//		printf("%d\n", AN_loop);
		//	}
		//}
		//if(tarFlg==77){  //i
		//	if(AN_loop < 40){
		//		DA_data[0] = 0.4*g[0];
		//		DA_data[1] = -0.4*g[1];
		//		DA_data[4] = 3.5/*6.0*//*-3.5*/;
		//		DA_data[5] = 4.0/*0.0*/;
		//	}else if(AN_loop > 40){
		//		DA_data[0] = 0.4*g[0];
		//		DA_data[1] = -0.4*g[1];
		//		DA_data[2] = 0.0;
		//		DA_data[3] = 0.0;
		//		DA_data[4] = 0.0/*-3.5*/;
		//		DA_data[5] = 0.0;
		//	}
		//}
	AN_loop++;

	if(tarFlg==6){	//s
		for(int i=0; i<DA_ChNum; i++){
			DA_data[i] = 0.0;
		}
		printf("STOP!!!");
	}

	AioMultiAoEx(DA_Id, DA_ChNum, DA_data);	//DA�o��

	end = clock();	//���Ԏ擾

	if(tarFlg == 0 || tarFlg == 1 || tarFlg== 6|| tarFlg==8 || tarFlg==9 || tarFlg==10 || tarFlg==70 || tarFlg==77 || tarFlg==78 || tarFlg==79 || tarFlg==100 || tarFlg==80 || tarFlg==81 || tarFlg==90){
		fprintf_s(fp0, "%I64d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			diffTime,(double)(end-start0)/CLOCKS_PER_SEC,
			KPe[0] * ( qd[0] - fq[0] )/10.0, KPe[1] * ( qd[1] - fq[1] )/10.0, KPe[2] * ( qd[2] - fq[2] )/10.0, KPe[3] * ( qd[3] - fq[3] )/10.0,
			KVe[0] * (qd_dot[0] - fq_dot[0])/10.0,  KVe[1] * (qd_dot[1] - fq_dot[1])/10.0,  -1.0 * KVe[2] * fq_dot[2]/10.0, KVe[3] * (qd_dot[3] - fq_dot[3])/10.0,
			KIe[0] * sum_q[0]/10.0, KIe[1] * sum_q[1]/10.0, KIe[2] * sum_q[2]/10.0, KIe[3] * sum_q[3]/10.0,
			KPc[0]*(J[0]*diff1.x + J[4]*diff1.y + J[8]*diff1.z)/10.0, KPc[1]*(J[1]*diff1.x + J[5]*diff1.y + J[9]*diff1.z)/10.0,
			KPc[2]*(J[2]*diff1.x + J[6]*diff1.y + J[10]*diff1.z)/10.0, KPc[3]*(J[3]*diff1.x + J[7]*diff1.y + J[11]*diff1.z)/10.0,
			KIc[0]*sum_diff[0]/10.0, KIc[1]*sum_diff[1]/10.0, KIc[2]*sum_diff[2]/10.0, KIc[3]*sum_diff[3]/10.0,
			DA_data[0]/10.0, DA_data[1]/10.0, DA_data[2]/10.0, DA_data[3]/10.0,DA_data[4]/10.0);

		fprintf(fp1,"%I64d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			diffTime, (double)(end-start0)/CLOCKS_PER_SEC,
			DA_data[0]/10.0, DA_data[1]/10.0, DA_data[2]/10.0, DA_data[3]/10.0, DA_data[4]/10.0, DA_data[5]/10.0, DA_data[6]/10.0, DA_data[7]/10.0, DA_data[8]/10.0,
			fq[0],fq[1],fq[2]/M_PI, fq[3], fq[4]/M_PI, fq[5]/M_PI, fqq[6], qd[0], qd[1], qd[2]/M_PI, qd[3], qd[4]/M_PI, qd[5]/M_PI, qd[6],
			fq_dot[0], fq_dot[1],fq_dot[2], fq_dot[3], fq_dot[4], q_dot[5], qd_dot[0],qd_dot[1],qd_dot[2],qd_dot[3], qd_dot[4], qd_dot[5],
			diff1.x,diff1.y,diff1.z,
			diff1.x*1.218,diff1.y*0.843,diff1.z*0.843,
			sqrt(diff1.x*diff1.x*1.218*1.218 + diff1.y*diff1.y*0.843*0.843 + diff1.z*diff1.z*0.843*0.843),
			a[0]*g[0]/10.0, -1.0*(a[1]*g[1]/10.0));
	}
	//AN_loop++;
	loop++;
}

void CALLBACK TimerProc(UINT wID,UINT uMsg,DWORD dwUser,DWORD dw1,DWORD dw2){

	control();

}

//���[�p�X�t�B���^
double LOWPASS(double src, double prev){
	double dst = 0.0;
	double cutoff = 100.0;

	dst = ( prev + cutoff * src * 0.001 ) / ( 1.0 + cutoff * 0.001 );
	prev = dst;

	return dst;
}

double LOWPASS2(double src2, double prev2){
	double dst2 = 0.0;
	double cutoff2 =100;

	dst2 = ( prev2 + cutoff2 * src2 * 0.001 ) / ( 1.0 + cutoff2 * 0.001 );
	prev2 = dst2;

	return dst2;
}