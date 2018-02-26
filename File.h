#if !defined(File_h)
	#define  File_h

#include<stdio.h>
#include<vector>


class File{
private:

	double SamplingTime;		//サンプリングタイム
	int LoopNumber;				//LOOP回数
	FILE *fname;

public:

	
	File(double St, int lnum);	//コンストラクタ
	void Fileout( char rfname[100], std::vector<double> a);					//ファイル出力
	void Fileout( char rfname[100], std::vector<double> a, std::vector<double> b);					//ファイル出力
	void Fileout( char rfname[100], std::vector<double> a, std::vector<double> b, std::vector<double> c);					//ファイル出力
	

};

#endif
