#if !defined(File_h)
	#define  File_h

#include<stdio.h>
#include<vector>


class File{
private:

	double SamplingTime;		//�T���v�����O�^�C��
	int LoopNumber;				//LOOP��
	FILE *fname;

public:

	
	File(double St, int lnum);	//�R���X�g���N�^
	void Fileout( char rfname[100], std::vector<double> a);					//�t�@�C���o��
	void Fileout( char rfname[100], std::vector<double> a, std::vector<double> b);					//�t�@�C���o��
	void Fileout( char rfname[100], std::vector<double> a, std::vector<double> b, std::vector<double> c);					//�t�@�C���o��
	

};

#endif
