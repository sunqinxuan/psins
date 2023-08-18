#include <iostream>
#include <fstream>
#include "PSINS.h"

class CFileIMU6:public CFileRdWt  // read the 6-column IMU file created by 'imufile.m'
{
public:
	CVect3 *pwm, *pvm, att0, vn0, pos0, gf, af;
	double t0, ts, g0, t;
	CFileIMU6(const char *fname0):CFileRdWt(fname0, 6) {
		CFileRdWt::load(1);	att0=*(CVect3*)&buff[0]*glv.deg, vn0=*(CVect3*)&buff[3];
		CFileRdWt::load(1);	pos0=LLH(buff[0],buff[1],buff[2]);	t0=buff[3], ts=buff[4]/1000.0, g0=buff[5];
		CFileRdWt::load(1);	gf=*(CVect3*)&buff[0]*glv.sec, af=*(CVect3*)&buff[3]*g0*1.0e-6;
		pwm=(CVect3*)&buff[0], pvm=(CVect3*)&buff[3];
		t = t0;
	};
	int load(int lines=1, BOOL txtDelComma=1) {
		if(!CFileRdWt::load(lines)) return 0;
		buff[0]*=gf.i, buff[1]*=gf.j, buff[2]*=gf.k;
		buff[3]*=af.i, buff[4]*=af.j, buff[5]*=af.k;
		t += ts;
		return 1;
	};
};

int main(void)
{
//	CMat3 mat3;
//	std::cout<<&mat3.e00<<","<<&mat3.e00+1<<std::endl;
//	std::cout<<&mat3.e01<<","<<&mat3.e01+1<<std::endl;
//	std::cout<<&mat3.e02<<","<<&mat3.e02+1<<std::endl;
//	std::cout<<&mat3.e10<<","<<&mat3.e10+1<<std::endl;
//	std::cout<<&mat3.e11<<","<<&mat3.e11+1<<std::endl;
//	std::cout<<&mat3.e12<<","<<&mat3.e12+1<<std::endl;
//	std::cout<<&mat3.e20<<","<<&mat3.e20+1<<std::endl;
//	std::cout<<&mat3.e21<<","<<&mat3.e21+1<<std::endl;
//	std::cout<<&mat3.e22<<","<<&mat3.e22+1<<std::endl;
//	return 1;

	CFileRdWt::Dir("../Data/");
	std::cout<<CFileRdWt::dirIn<<std::endl;
	std::cout<<CFileRdWt::dirOut<<std::endl;
	CFileIMU6 fimu("lasergyro.imu"); 
	CFileRdWt faln("aln.bin"), fins("ins.bin");
//	CAligni0 aln(pos0);
	std::cout<<"fimu.ts="<<fimu.ts<<std::endl;
	std::cout<<"fimu.att0="<<fimu.att0.i<<","<<fimu.att0.j<<","<<fimu.att0.k<<std::endl;
	std::cout<<"fimu.vn0="<<fimu.vn0.i<<","<<fimu.vn0.j<<","<<fimu.vn0.k<<std::endl;
	std::cout<<"fimu.pos0="<<fimu.pos0.i<<","<<fimu.pos0.j<<","<<fimu.pos0.k<<std::endl;
	CSINS sins0(fimu.att0,fimu.vn0,fimu.pos0);
	std::cout<<"sins0"<<std::endl;
	CAlignkf aln(sins0,fimu.ts);
	CSINS sins;
	std::cout<<"sins"<<std::endl;
	int alnOK=0;
	std::ofstream fp_aln,fp_ins;
	fp_aln.open("aln.txt",std::ios::out);
	fp_ins.open("ins.txt",std::ios::out);
	for(int i=0; i<2000*100; i++)
	{
		if(!fimu.load(1)) break;
//		if(i>600*100)
//		{
//			if(!alnOK) {
//				alnOK=1;
//				sins.Init(aln.qnb, O31, fimu.pos0, aln.kftk);
//			}
//			else {
//				sins.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
//				fins<<sins;
//				fp_ins<<sins.att.i<<" "<<sins.att.j<<" "<<sins.att.k<<" "
//					<<sins.vn.i<<" "<<sins.vn.j<<" "<<sins.vn.k<<" "
//					<<sins.pos.i<<" "<<sins.pos.j<<" "<<sins.pos.k<<" "
//					<<sins.eb.i<<" "<<sins.eb.j<<" "<<sins.eb.k<<" "
//					<<sins.db.i<<" "<<sins.db.j<<" "<<sins.db.k<<" "
//					<<sins.tk<<std::endl;;
//			}
//		}
//		else 
		{
			aln.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
			CVect3 att=q2att(aln.qnb);
//			std::cout<<att.i<<" "<<att.j<<" "<<att.k<<" "<<aln.kftk<<std::endl;
			fp_aln<<att.i<<" "<<att.j<<" "<<att.k<<" "<<aln.kftk<<std::endl;
			faln<<q2att(aln.qnb)<<aln.kftk;
		}
		disp(i, 100, 100);
	}
	fp_aln.close();
	fp_ins.close();
	std::cout<<"main return"<<std::endl;
	return 0;
}

