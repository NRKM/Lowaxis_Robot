
#include"File.h"



File::File(double St, int lnum){

	SamplingTime=St;
	LoopNumber = lnum;
	
}


void File::Fileout(char rfname[100], std::vector<double> a){


	fname=fopen(rfname,"w");  //�@�o�̓t�@�C�����I�[�v��
	if (fname==NULL){        //�@�G���[����

		perror("�G���[�I�o�̓t�@�C�����I�[�v���ł��܂���\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//�f�[�^�̏o��

		fprintf(fname,"%lf\t%lf\n",i*SamplingTime,a[i]);
		
	}

	//�o�̓t�@�C�����N���[�Y
	if(fclose(fname)!=NULL){
	
		perror("�G���[�I�o�̓t�@�C�����N���[�Y�ł��܂���\n");
	       	
	}

}

void File::Fileout( char rfname[100], std::vector<double> a, std::vector<double> b){
	

	fname=fopen(rfname,"w");  //�@�o�̓t�@�C�����I�[�v��
	if (fname==NULL){        //�@�G���[����

		perror("�G���[�I�o�̓t�@�C�����I�[�v���ł��܂���\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//�f�[�^�̏o��

		fprintf(fname,"%lf\t%lf\t%lf\n",i*SamplingTime,a[i],b[i]);
		
	}

	//�o�̓t�@�C�����N���[�Y
	if(fclose(fname)!=NULL){
	
		perror("�G���[�I�o�̓t�@�C�����N���[�Y�ł��܂���\n");
	       	
	}

}

void File::Fileout( char rfname[100], std::vector<double> a, std::vector<double> b, std::vector<double> c){
	

	fname=fopen(rfname,"w");  //�@�o�̓t�@�C�����I�[�v��
	if (fname==NULL){        //�@�G���[����

		perror("�G���[�I�o�̓t�@�C�����I�[�v���ł��܂���\n");
        exit(1);	

	}

	for(int i=0;i<LoopNumber;i++){		//�f�[�^�̏o��

		fprintf(fname,"%lf\t%lf\t%lf\t%lf\n",i*SamplingTime,a[i],b[i],c[i]);
		
	}

	//�o�̓t�@�C�����N���[�Y
	if(fclose(fname)!=NULL){
	
		perror("�G���[�I�o�̓t�@�C�����N���[�Y�ł��܂���\n");
	       	
	}

}

