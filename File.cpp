
#include"File.h"



File::File(double St, int lnum){

	SamplingTime=St;
	LoopNumber = lnum;
	
}


void File::Fileout(char rfname[100], std::vector<double> a){


	fname=fopen(rfname,"w");  //　出力ファイルをオープン
	if (fname==NULL){        //　エラー処理

		perror("エラー！出力ファイルをオープンできません\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//データの出力

		fprintf(fname,"%lf\t%lf\n",i*SamplingTime,a[i]);
		
	}

	//出力ファイルをクローズ
	if(fclose(fname)!=NULL){
	
		perror("エラー！出力ファイルをクローズできません\n");
	       	
	}

}

void File::Fileout( char rfname[100], std::vector<double> a, std::vector<double> b){
	

	fname=fopen(rfname,"w");  //　出力ファイルをオープン
	if (fname==NULL){        //　エラー処理

		perror("エラー！出力ファイルをオープンできません\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//データの出力

		fprintf(fname,"%lf\t%lf\t%lf\n",i*SamplingTime,a[i],b[i]);
		
	}

	//出力ファイルをクローズ
	if(fclose(fname)!=NULL){
	
		perror("エラー！出力ファイルをクローズできません\n");
	       	
	}

}

void File::Fileout( char rfname[100], std::vector<double> a, std::vector<double> b, std::vector<double> c){
	

	fname=fopen(rfname,"w");  //　出力ファイルをオープン
	if (fname==NULL){        //　エラー処理

		perror("エラー！出力ファイルをオープンできません\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//データの出力

		fprintf(fname,"%lf\t%lf\t%lf\t%lf\n",i*SamplingTime,a[i],b[i],c[i]);
		
	}

	//出力ファイルをクローズ
	if(fclose(fname)!=NULL){
	
		perror("エラー！出力ファイルをクローズできません\n");
	       	
	}

}

