////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Author:      GU Zhaopeng <zpgu@nlpr.ia.ac.cn>
// Version:     0.0.1
// Date:        2010-10                                         
// Description:   This is an implementation of the classic P3P(Perspective 3-Points) algorithm problem solution 
//				  in the Ransac paper "M. A. Fischler, R. C. Bolles. Random Sample Consensus: A Paradigm for 
//                Model Fitting with Applications to Image Analysis and Automated Cartography. Comm. of the ACM, 
//                Vol 24, pp 381-395, 1981.". The algorithm gives the four probable solutions of the P3P problem 
//                in about 0.1ms, and can be used as input of the consequent RANSAC step. The codes needs the 
//                numerics library VNL which is a part of the widely used computer vision library VXL. One can 
//                download and install it from http://vxl.sourceforge.net/.
//
//
// Copyright (C) 2009-2010 OpenPR 
// All rights reserved. 
// 
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met: 
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimer. 
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimer in the 
//       documentation and/or other materials provided with the distribution. 
//     * Neither the name of OpenPR nor the names of its  
//       contributors may be used to endorse or promote products derived 
//       from this software without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL HOLDER AND CONTRIBUTORS BE LIABLE FOR ANY 
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "P3PSolver.h"

P3PSolver::P3PSolver(void)
{
}

P3PSolver::~P3PSolver(void)
{
}

void P3PSolver::Initial_RANSAC(void)
{
	W = cvCreateMat( 3, 3, CV_64FC1 );
	cvSetZero(W);
	I = cvCreateMat( 3, 2, CV_64FC1 );
	cvSetZero(I);
}
void P3PSolver::SetIntrinsic(double f_u,double f_v,double u_0,double v_0,double k_1,double k_2,double p_1,double p_2)
{
	
	//double k1,k2,p1,p2;
	fu=f_u;
	fv=f_v;
	u0=u_0;
	v0=v_0;
	//k1=k_1;
	//k2=k_2;
	//p1=p_1;
	//p2=p_2;
}

void P3PSolver::SetIntrinsic(double *param)
{
	double* p=param;
	fu=*p;p++;
	fv=*p;p++;
	u0=*p;p++;
	v0=*p;p++;
	//k1=*p;p++;
	//k2=*p;p++;
	//p1=*p;p++;
	//p2=*p;p++;
}

void P3PSolver::SetPointsCorrespondance(double *ptw,double *pti)
{
	cvSetZero(W);
	cvSetZero(I);
	for(int i=0 ; i<3 ;i++)
	{
		cvmSet( W , i, 0 , *(ptw+i*3+0));
		cvmSet( W , i, 1 , *(ptw+i*3+1));
		cvmSet( W , i, 2 , *(ptw+i*3+2));
		
		cvmSet( I , i, 0 , *(pti+i*2));
		cvmSet( I , i, 1 , *(pti+i*2+1));		
	}
}

void P3PSolver::GetCoeffcientofP3P(double Rab, double Rac, double Rbc, 
					   double Cab, double Cac, double Cbc,
					   double &G4, double &G3, double &G2, double &G1, double &G0)
{
	double K1=Rbc*Rbc/(Rac*Rac);
	double K2=Rbc*Rbc/(Rab*Rab);

	G4=(K1*K2-K1-K2)*(K1*K2-K1-K2)-4*K1*K2*Cbc*Cbc;

	G3=4*(K1*K2-K1-K2)*K2*(1-K1)*Cab+
		4*K1*Cbc*((K1*K2+K2-K1)*Cac+2*K2*Cab*Cbc);

	G2=(2*K2*(1-K1)*Cab)*(2*K2*(1-K1)*Cab)+
		2*(K1*K2+K1-K2)*(K1*K2-K1-K2)+
		4*K1*((K1-K2)*Cbc*Cbc+
		(1-K2)*K1*Cac*Cac-2*K2*(1+K1)*Cab*Cac*Cbc);

	G1=4*(K1*K2+K1-K2)*K2*(1-K1)*Cab+
		4*K1*((K1*K2-K1+K2)*Cac*Cbc+2*K1*K2*Cab*Cac*Cac);

	G0=(K1*K2+K1-K2)*(K1*K2+K1-K2)-4*K1*K1*K2*Cac*Cac;
}


void P3PSolver::Solve(vector<Matrix>& X , vector< Rectangular >& Rs , vector< Matrix >& ts)
{
//------------------------------------------宣告--------------------------------------------------------------------
	CvMat* W_one = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* W_two = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* W_three = cvCreateMat( 1, 3, CV_64FC1 );

	CvMat* VAB = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* VAC = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* VBC = cvCreateMat( 1, 3, CV_64FC1 );

	CvMat* CA = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* CB = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* CC = cvCreateMat( 1, 3, CV_64FC1 );

	CvMat* WQ = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* WP = cvCreateMat( 1, 3, CV_64FC1 );
		
	CvMat* P1 = cvCreateMat( 1, 4, CV_64FC1 );
	CvMat* P2 = cvCreateMat( 1, 4, CV_64FC1 );
	CvMat* P3 = cvCreateMat( 1, 4, CV_64FC1 );

	CvMat* NP1 = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* NP2 = cvCreateMat( 1, 3, CV_64FC1 );
	CvMat* NP3 = cvCreateMat( 1, 3, CV_64FC1 );


	CvMat* VCX = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* VCY = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* VCZ = cvCreateMat( 1 , 3 , CV_64FC1 );

	CvMat* Vla = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* Vlb = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* Vlc = cvCreateMat( 1 , 3 , CV_64FC1 );

	CvMat* WA1 = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* WB1 = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* WC1 = cvCreateMat( 1 , 3 , CV_64FC1 );

	CvMat* vcx = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* vcy = cvCreateMat( 1 , 3 , CV_64FC1 );
	CvMat* vcz = cvCreateMat( 1 , 3 , CV_64FC1 );

	CvMat* BIGA = cvCreateMat( 3 , 4 , CV_64FC1 );			
		
	CvMat* BIGA_W = cvCreateMat( 3 , 4 , CV_64FC1 );
	CvMat* BIGA_U = cvCreateMat( 3 , 3 , CV_64FC1 );
	CvMat* BIGA_V = cvCreateMat( 4 , 4 , CV_64FC1 );		
	
	CvMat* WR = cvCreateMat( 1 , 3 , CV_64FC1 );
	
	CvMat* WL = cvCreateMat( 1 , 3 , CV_64FC1 );

	CvMat* R1 = cvCreateMat( 3 , 3 , CV_64FC1 );
	CvMat* R2 = cvCreateMat( 3 , 3 , CV_64FC1 );
	CvMat* R3 = cvCreateMat( 3 , 3 , CV_64FC1 );

	CvMat* Rext = cvCreateMat( 3 , 3 , CV_64FC1 );
	
	CvMat* text = cvCreateMat( 3 , 1 , CV_64FC1 );

//------------------------------------------------------------------------------------------------------------------

	fstream app_time("txt\\p3p_time.txt",ios::app);

	double time1,time2,time3,time4,time5,time6;
	double timeNorm_1 , timeNorm_2 , timeCopy_1 , timeCopy_2 , time_Rext_1 , time_Rext_2 , time_WL_1 , time_WL_2;

	fstream app_point("txt\\point.txt",ios::app);
	app_point << "ok4_1_6_1" << endl;

timeCopy_1 = (double)cvGetTickCount()/cvGetTickFrequency();
	MatrixCopy(W , W_one   , 0 , 0 , 0 , 0 , 3 , 1);
timeCopy_2 = (double)cvGetTickCount()/cvGetTickFrequency();
	app_time << "Copy  =  " << timeCopy_2 - timeCopy_1 << endl;

	MatrixCopy(W , W_two   , 0 , 1 , 0 , 0 , 3 , 1);
	MatrixCopy(W , W_three , 0 , 2 , 0 , 0 , 3 , 1);

	//yes_no = false;
	//fstream app_WL("WL.txt",ios::app);
	//Compute Rab Rac Rbc in world frame

	cvmSub(W_two , W_one , VAB);
	cvmSub(W_three , W_one , VAC);
	cvmSub(W_three , W_two , VBC);

	//Length of edge between control points

	double Rab  = cvNorm(VAB,0,CV_DIFF_L2);		//cvNorm(輸入CvMat資料結構,輸入CvMat資料結構或為空,參數或代號,輸入CvMat遮罩)
	double Rac  = cvNorm(VAC,0,CV_DIFF_L2);
	double Rbc  = cvNorm(VBC,0,CV_DIFF_L2);

	cvReleaseMat(&VBC);

	app_point << "ok4_1_6_2" << endl;
	
	
	//Get norm of each ray in camera frame
	//CA0=[(IA(1,1)-u0)/fu;(IA(2,1)-v0)/fv;1];
	//CB0=[(IB(1,1)-u0)/fu;(IB(2,1)-v0)/fv;1];
	//CC0=[(IC(1,1)-u0)/fu;(IC(2,1)-v0)/fv;1];


	cvmSet( CA , 0, 0 , (cvmGet(I , 0 , 0) -u0)/fu );
	cvmSet( CA , 0, 1 , (cvmGet(I , 0 , 1) -v0)/fv );
	cvmSet( CA , 0, 2 , 1 );

	cvmSet( CB , 0, 0 , (cvmGet(I , 1 ,0) -u0)/fu );
	cvmSet( CB , 0, 1 , (cvmGet(I , 1 ,1) -v0)/fv );
	cvmSet( CB , 0, 2 , 1 );

	cvmSet( CC , 0, 0 , (cvmGet(I , 2 ,0) -u0)/fu );
	cvmSet( CC , 0, 1 , (cvmGet(I , 2 ,1) -v0)/fv );
	cvmSet( CC , 0, 2 , 1 );
/*
	cout << cvmGet(CA , 0 , 0) << "  " << cvmGet(CA , 0 , 1) << "  " << cvmGet(CA , 0 , 2) << endl;
	cout << cvmGet(CB , 0 , 0) << "  " << cvmGet(CB , 0 , 1) << "  " << cvmGet(CB , 0 , 2) << endl;
	cout << cvmGet(CC , 0 , 0) << "  " << cvmGet(CC , 0 , 1) << "  " << cvmGet(CC , 0 , 2) << endl;
*/

timeNorm_1 = (double)cvGetTickCount()/cvGetTickFrequency();
	Norm(CA , CA);
timeNorm_2 = (double)cvGetTickCount()/cvGetTickFrequency();
	app_time << "Norm  =  " << timeNorm_2 - timeNorm_1 << endl;
	Norm(CB , CB);
	Norm(CC , CC);

	app_point << "ok4_1_6_3" << endl;
	
	
	double Rab1  = cvNorm(CB,CA,CV_DIFF_L2);
	double Rac1  = cvNorm(CC,CA,CV_DIFF_L2);
	double Rbc1  = cvNorm(CC,CB,CV_DIFF_L2);
	
	//Cosine of angles
	double Calb,Calc,Cblc;

	//Compute Calb Calc Cblc using Law of Cosine
	
	Calb=(2-Rab1*Rab1)/2;
	Calc=(2-Rac1*Rac1)/2;
	Cblc=(2-Rbc1*Rbc1)/2;


	//===========================================================================================
	//Solve equation!
	//===========================================================================================
	
	//Get coefficients of the quartic polynomial equation
	double G4,G3,G2,G1,G0;
	GetCoeffcientofP3P(Rab,Rac,Rbc,Calb,Calc,Cblc,G4,G3,G2,G1,G0);
	//Solve it using VNL
	double coeff[5];
	coeff[0]=G4;
	coeff[1]=G3;
	coeff[2]=G2;
	coeff[3]=G1;
	coeff[4]=G0;

	app_point << "ok4_1_6_4" << endl;
	

	int root_number = 0;
	double	real_roots[4] = {0,0,0,0};

time1 = (double)cvGetTickCount()/cvGetTickFrequency();
	root_number = solve_deg4(G4,G3,G2,G1,G0,real_roots[0],real_roots[1],real_roots[2],real_roots[3]);
time2 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "solve_deg4 =  " << time2 - time1 << endl;


	//Temp roots
	//Format: [[x0 a0 b0] [x1 a1 b1]...]
	vector< Matrix > rootxab;
	Matrix r;
	for (int i=0;i<root_number;i++)
	{
		double x=real_roots[i];
		double ifadd=true;
		for (int j=0;j<rootxab.size();j++)
		{
			if (abs(rootxab[j].data_[0]-x)<Epsilon)
			{
				ifadd=false;
			}
		}
		if (ifadd)
		{
			memset(&r , 0 , sizeof(Matrix));
			//Compute a b
			double a=Rab/sqrt(x*x-2*x*Calb+1);
			double b=a*x;
			r.data_[0]=x;
			r.data_[1]=a;
			r.data_[2]=b;
			rootxab.push_back(r);
		}
	}

	app_point << "ok4_1_6_5" << endl;
	

	double K1=Rbc*Rbc/(Rac*Rac);
	double K2=Rbc*Rbc/(Rab*Rab);

	vector< root > roots;

	for (int i=0;i<rootxab.size();i++)
	{
		double delta;
		double m0,m1,p0,p1,q0,q1;

		double x=rootxab[i].data_[0];
		double a=rootxab[i].data_[1];
		double b=rootxab[i].data_[2];

		m0=1-K1;	p0=2*(K1*Calc-x*Cblc);		q0=x*x-K1;
		m1=1;		p1=-2*x*Cblc;				q1=x*x*(1-K2)+2*x*K2*Calb-K2;

		delta=abs(m1*q0-m0*q1);

		
		if (delta<Epsilon)
		{
			//delta==0
			double y_candidate[2];
			y_candidate[0]=Calc+sqrt(Calc*Calc+(Rac*Rac-a*a)/(a*a));
			y_candidate[1]=Calc-sqrt(Calc*Calc+(Rac*Rac-a*a)/(a*a));

			double c_candidate[2];
			c_candidate[0]=y_candidate[0]*a;
			c_candidate[1]=y_candidate[1]*a;

			for (int j=0;j<2;j++)
			{
				double delta1=abs(b*b+c_candidate[j]*c_candidate[j]-2*b*c_candidate[j]*Cblc-Rbc*Rbc);
				if (delta1<Epsilon)
				{
					double y=y_candidate[j];
					double c=c_candidate[j];
					root r;
					r.data_[0]=a;
					r.data_[1]=b;
					r.data_[2]=c;
					r.data_[3]=x;
					r.data_[4]=y;
					roots.push_back(r);
				}
			}
		}
		else
		{
			//delta!=0
			double y=(p1*q0-p0*q1)/(m0*q1-m1*q0);
			double c=y*a;
			root r;
			r.data_[0]=a;
			r.data_[1]=b;
			r.data_[2]=c;
			r.data_[3]=x;
			r.data_[4]=y;
			roots.push_back(r);
		}
	}

time3 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "solve a b c x y  =   " << time3 - time2 << endl;

	app_point << "ok4_1_6_6" << endl;
	

	rootxab.swap(vector< Matrix >());

//app_WL << (time2 - time1)/(cvGetTickFrequency()*1000000.) << endl;

	//===========================================================================================
	//Get Extrinsics!
	//===========================================================================================

	for (int i=0;i<roots.size();i++)
	{
time_WL_1 = (double)cvGetTickCount()/cvGetTickFrequency();	

		app_point << "ok4_1_6_7" << endl;


		double a=roots[i].data_[0];
		double b=roots[i].data_[1];
		double c=roots[i].data_[2];

		//%Get cosine of the angles
		//Clab=(a^2+Rab^2-b^2)/(2*a*Rab);
		//Clac=(a^2+Rac^2-c^2)/(2*a*Rac);
		
		double Clab=(a*a+Rab*Rab-b*b)/(2*a*Rab);
		double Clac=(a*a+Rac*Rac-c*c)/(2*a*Rac);

		//%Get scale along norm vector
		//Raq=a*Clab;
		//Rap=a*Clac;

		double Raq=a*Clab;
		double Rap=a*Clac;

		//%Get norm vector of plane P1 P2
		//VAC=WC-WA;
		//VAB=WB-WA;
		double VAB_norm = cvNorm(VAB,0,CV_DIFF_L2);
		double VAC_norm = cvNorm(VAC,0,CV_DIFF_L2);


		for(int k=0 ; k<3 ; k++)
		{
			cvmSet(WQ , 0 , k , cvmGet(W_one,0,k)+Raq*cvmGet(VAB,0,k)/VAB_norm);
			cvmSet(WP , 0 , k , cvmGet(W_one,0,k)+Rap*cvmGet(VAC,0,k)/VAC_norm);
		}


		app_point << "ok4_1_6_8" << endl;


		double DP1,DP2,DP3;

		//%Compute Plane P1 P2 P3
		//NP1=VAB/norm(VAB);
		//DP1=-NP1'*WQ;
		//P1=[NP1;DP1];

time4 = (double)cvGetTickCount()/cvGetTickFrequency();

		Norm(VAB,NP1);
		DP1 = cvDotProduct(NP1,WQ);			//內積
		MatrixCopy(NP1 , P1 , 0 , 0 , 0 , 0 , 3 , 1);
		cvmSet(P1 , 0 , 3 , -DP1 );



		//NP2=VAC/norm(VAC);
		//DP2=-NP2'*WP;
		//P2=[NP2;DP2];

		Norm(VAC,NP2);
		DP2 = cvDotProduct(NP2,WP);			//內積
		MatrixCopy(NP2 , P2 , 0 , 0 , 0 , 0 , 3 , 1);
		cvmSet(P2 , 0 , 3 , -DP2 );


		//%Current we use ACxAB get the norm of P3
		//NP3=crossproduct3d(VAC,VAB)';
		//NP3=NP3/norm(NP3);
		//DP3=-NP3'*WA;
		//P3=[NP3;DP3];


		cvCrossProduct(VAC , VAB , NP3);	//外積

		app_point << "ok4_1_6_9" << endl;
			
		double NP3_norm= cvNorm(NP3,0,CV_DIFF_L2);
		if(NP3_norm == 0) break;
		Norm(NP3,NP3);
		DP3 = cvDotProduct(NP3,W_one);
		MatrixCopy(NP3 , P3 , 0 , 0 , 0 , 0 , 3 , 1);
		cvmSet(P3 , 0 , 3 , -DP3);

time5 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "P1~P3 =    " << time5 - time4 << endl;

		//BIGA=[P1';P2';P3'];

		MatrixCopy(P1 , BIGA , 0 , 0 , 0 , 0 , 4 , 1);
		MatrixCopy(P2 , BIGA , 0 , 0 , 0 , 1 , 4 , 1);
		MatrixCopy(P3 , BIGA , 0 , 0 , 0 , 2 , 4 , 1);


		app_point << "ok4_1_6_10" << endl;


		cvSVD(BIGA , BIGA_W , BIGA_U , BIGA_V);

		//WR=null(BIGA,3);

		cvmSet(WR , 0 , 0 , cvmGet(BIGA_V , 0 , 3 ) / cvmGet(BIGA_V , 3 , 3 ));
		cvmSet(WR , 0 , 1 , cvmGet(BIGA_V , 1 , 3 ) / cvmGet(BIGA_V , 3 , 3 ));
		cvmSet(WR , 0 , 2 , cvmGet(BIGA_V , 2 , 3 ) / cvmGet(BIGA_V , 3 , 3 ));


time6 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "solve_BIGA =    " << time6 - time5 << endl;

		//%Get length of LR
		//Rar=norm(WA-WR);
		//Rlr=sqrt(a^2-Rar^2);

		app_point << "ok4_1_6_11" << endl;			

		double Rar,Rlr;
		Rar = cvNorm(W_one,WR,CV_DIFF_L2);
		if((a*a-Rar*Rar) < 0)	continue;
		Rlr=sqrt(a*a-Rar*Rar);

		//%Get Position of L in world frame
		//WL=WR+NP3*Rlr;

		
		cvmSet(WL , 0 , 0 ,cvmGet(WR , 0 , 0) + cvmGet(NP3 , 0 , 0)*Rlr);
		cvmSet(WL , 0 , 1 ,cvmGet(WR , 0 , 1) + cvmGet(NP3 , 0 , 1)*Rlr);
		cvmSet(WL , 0 , 2 ,cvmGet(WR , 0 , 2) + cvmGet(NP3 , 0 , 2)*Rlr);

time_WL_2 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "solve_WL =    " << time_WL_2 - time_WL_1 << endl;


		app_point << "ok4_1_6_12" << endl;
			

		//%Get unprojection ray of image points
		//CA0=[(IA(1,1)-u0)/fu;(IA(2,1)-v0)/fv;1];
		//CB0=[(IB(1,1)-u0)/fu;(IB(2,1)-v0)/fv;1];
		//CC0=[(IC(1,1)-u0)/fu;(IC(2,1)-v0)/fv;1];

		//VCx=CB1-CA1;
		//VCy=CC1-CA1;
		//VCz=crossproduct3d(VCx,VCy)';
		//VCy=crossproduct3d(VCz,VCx)';

		//VCx=VCx/norm(VCx);
		//VCy=VCy/norm(VCy);
		//VCz=VCz/norm(VCz);

time_Rext_1 = (double)cvGetTickCount()/cvGetTickFrequency();
		cvmSub(CB , CA , VCX);
		cvmSub(CC , CA , VCY);

		cvCrossProduct(VCX , VCY , VCZ);
		cvCrossProduct(VCZ , VCX , VCY);

		Norm(VCX,VCX);
		Norm(VCY,VCY);
		Norm(VCZ,VCZ);

		app_point << "ok4_1_6_13" << endl;			


		//%Get ray in the world frame
		//Vla=WA-WL;
		//Vlb=WB-WL;
		//Vlc=WC-WL;

		cvmSub(W_one   , WL , Vla);
		cvmSub(W_two   , WL , Vlb);
		cvmSub(W_three , WL , Vlc);

	
		Norm(Vla , Vla);
		Norm(Vlb , Vlb);
		Norm(Vlc , Vlc);

		app_point << "ok4_1_6_14" << endl;
			

		//WA1=WL+1*Vla;
		//WB1=WL+1*Vlb;
		//WC1=WL+1*Vlc;



		cvmAdd(WL , Vla , WA1);
		cvmAdd(WL , Vlb , WB1);
		cvmAdd(WL , Vlc , WC1);

		
		//vcx=WB1-WA1;
		//vcy=WC1-WA1;
		//vcz=crossproduct3d(vcx,vcy)';
		//vcy=crossproduct3d(vcz,vcx)';

		//vcx=vcx/norm(vcx);
		//vcy=vcy/norm(vcy);
		//vcz=vcz/norm(vcz);


		cvmSub(WB1 ,  WA1 , vcx);
		cvmSub(WC1 ,  WA1 , vcy);

		
		cvCrossProduct(vcx , vcy , vcz);
		cvCrossProduct(vcz , vcx , vcy);
		
		Norm(vcx , vcx);
		Norm(vcy , vcy);
		Norm(vcz , vcz);

		app_point << "ok4_1_6_15" << endl;
			


		//R=[VCx VCy VCz]*inv([vcx vcy vcz]);


		MatrixCopy(VCX , R1 , 0 , 0 , 0 , 0 , 3 , 1);
		MatrixCopy(VCY , R1 , 0 , 0 , 0 , 1 , 3 , 1);
		MatrixCopy(VCZ , R1 , 0 , 0 , 0 , 2 , 3 , 1);


		cvTranspose(R1,R1);

		MatrixCopy(vcx , R2 , 0 , 0 , 0 , 0 , 3 , 1);
		MatrixCopy(vcy , R2 , 0 , 0 , 0 , 1 , 3 , 1);
		MatrixCopy(vcz , R2 , 0 , 0 , 0 , 2 , 3 , 1);

		cvmMul(R1 , R2 , Rext);




		for(int k=0 ; k< 3 ; k++)
		{
			cvmSet(text , k , 0 , -cvmGet(WL , 0 , k));
		}
		cvmMul(Rext , text , text);

		app_point << "ok4_1_6_16" << endl;
			
time_Rext_2 = (double)cvGetTickCount()/cvGetTickFrequency();	
	app_time << "Rext  =   " << time_Rext_2 - time_Rext_1 << endl << endl;	

//--------------------------------------------------------------------------------------------

		Matrix temp;
		memset( &temp , 0 , sizeof(Matrix) );

		temp.data_[0] = cvmGet(WL , 0 , 0);
		temp.data_[1] = cvmGet(WL , 0 , 1);
		temp.data_[2] = cvmGet(WL , 0 , 2);

		X.push_back(temp);

		app_point << "ok4_1_6_17" << endl;
			
		
//---------------------------------------------------------

		Rectangular temp2;
		memset( &temp2 , 0 , sizeof(Rectangular) );

		for(int k=0 ; k<3 ; k++)
		{
			temp2.data_[k][0] = cvmGet(Rext , k , 0);
			temp2.data_[k][1] = cvmGet(Rext , k , 1);
			temp2.data_[k][2] = cvmGet(Rext , k , 2);
		}

		Rs.push_back(temp2);

		app_point << "ok4_1_6_18" << endl;
			

//---------------------------------------------------------

		Matrix temp3;
		memset( &temp3 , 0 , sizeof(Matrix) );

		temp3.data_[0] = cvmGet(text , 0 , 0);
		temp3.data_[1] = cvmGet(text , 1 , 0);
		temp3.data_[2] = cvmGet(text , 2 , 0);

		ts.push_back(temp3);

		app_point << "ok4_1_6_19" << endl;
			

//--------------------------------------------------------------------------------------------	
	}

	app_time << endl << endl;

//------------------------------------------釋放--------------------------------------------------------------------
	cvReleaseMat(&CA);
	cvReleaseMat(&CB);
	cvReleaseMat(&CC);
	cvReleaseMat(&W_one);
	cvReleaseMat(&W_two);
	cvReleaseMat(&W_three);
	cvReleaseMat(&VAB);
	cvReleaseMat(&VAC);
	roots.swap(vector< root >());
	cvReleaseMat(&WQ);
	cvReleaseMat(&WP);
	cvReleaseMat(&P1);
	cvReleaseMat(&P2);
	cvReleaseMat(&P3);
	cvReleaseMat(&NP1);
	cvReleaseMat(&NP2);
	cvReleaseMat(&BIGA);
	cvReleaseMat(&BIGA_W);
	cvReleaseMat(&BIGA_U);
	cvReleaseMat(&BIGA_V);
	cvReleaseMat(&NP3);
	cvReleaseMat(&Vla);
	cvReleaseMat(&Vlb);
	cvReleaseMat(&Vlc);
	cvReleaseMat(&WA1);
	cvReleaseMat(&WB1);
	cvReleaseMat(&WC1);
	cvReleaseMat(&vcx);
	cvReleaseMat(&vcy);
	cvReleaseMat(&vcz);
	cvReleaseMat(&R1);
	cvReleaseMat(&R2);
	cvReleaseMat(&VCX);
	cvReleaseMat(&VCY);
	cvReleaseMat(&VCZ);
	cvReleaseMat(&WL);
	cvReleaseMat(&Rext);
	cvReleaseMat(&text);
		
//------------------------------------------------------------------------------------------------------------------
	app_point << "ok4_1_6_20" << endl;

	app_point.close();


	app_time.close();


}

void P3PSolver::Norm(CvMat* A , CvMat *B)
{
	double number_norm = cvNorm(A,0,CV_DIFF_L2);
	cvmSet( B , 0, 0 , cvmGet(A , 0 , 0) / number_norm);
	cvmSet( B , 0, 1 , cvmGet(A , 0 , 1) / number_norm);
	cvmSet( B , 0, 2 , cvmGet(A , 0 , 2) / number_norm);
}

void P3PSolver::MatrixCopy( CvMat *mat, CvMat *submat, const int x, const int y, const int subx, const int suby, const int width, const int height )
{
	for( int i=0 ; i < width ; i++ )
	{
		for( int j=0 ; j < height ; j++ )
		{
			cvmSet( submat, suby+j, subx+i, cvmGet( mat, y+j, x+i ) );	
		}
	}
}
int P3PSolver::computePoses( double*featureVectors ,double*worldPoints , vector<Matrix>&X, vector<Rectangular>& Rs )
{



	// Extraction of world points
	/*
	TooN::Vector<3> P1 = worldPoints.T()[0];
	TooN::Vector<3> P2 = worldPoints.T()[1];
	TooN::Vector<3> P3 = worldPoints.T()[2];
	*/

	CvMat *P1 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *P2 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *P3 = cvCreateMat( 3,1, CV_64FC1 );

	for(int i=0;i<3;i++)	//前一刻視線向量
	{
		cvmSet(P1 ,i,0, featureVectors[i]);
		cvmSet(P2 ,i,0 ,featureVectors[i+3]);
		cvmSet(P3 ,i,0, featureVectors[i+6]);
	}


	// Verification that world points are not colinear
	/*
	TooN::Vector<3> temp1 = P2 - P1;
	TooN::Vector<3> temp2 = P3 - P1;
	if(TooN::norm(temp1 ^ temp2) == 0)
		return -1;
	*/
	CvMat *temp1 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *temp2 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *temp3 = cvCreateMat( 3,1, CV_64FC1 );
	cvSub(P2,P1,temp1);
	cvSub(P3,P1,temp2);

	//cvmMul(temp1, temp2, temp3);
	//if(cvNorm(temp3,NULL,CV_L1)==0)return -1;
		
	// Extraction of feature vectors
	/*
	TooN::Vector<3> f1 = featureVectors.T()[0];
	TooN::Vector<3> f2 = featureVectors.T()[1];
	TooN::Vector<3> f3 = featureVectors.T()[2];
	*/
	CvMat *f1 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *f2 = cvCreateMat( 3,1, CV_64FC1 );
	CvMat *f3 = cvCreateMat( 3,1, CV_64FC1 );


	for(int i=0;i<3;i++)	//這一刻單位視線向量
	{
		cvmSet(f1 , i ,0, worldPoints[i]);
		cvmSet(f2 , i ,0, worldPoints[i+3]);
		cvmSet(f3 , i, 0 ,worldPoints[i+6]);
	}

	// Creation of intermediate camera frame
	/*
	TooN::Vector<3> e1 = f1;
	TooN::Vector<3> e3 = f1 ^ f2;
	e3 = e3 / TooN::norm(e3);
	TooN::Vector<3> e2 = e3 ^ e1;
	*/
	CvMat *e1 = cvCloneMat(f1);
	CvMat *e3 = cvCreateMat( 3, 1, CV_64FC1 );

	cvCrossProduct(f1,f2,e3); 
	


	//double norm = cvNorm(e3, NULL,CV_L1);
	double norm = sqrt(cvmGet(e3,0,0)*cvmGet(e3,0,0)+cvmGet(e3,1,0)*cvmGet(e3,1,0)+cvmGet(e3,2,0)*cvmGet(e3,2,0));


	CvMat *norm_mat = cvCreateMat( 3, 1, CV_64FC1 );
	for(int i=0;i<3;i++)
	{
		cvmSet(norm_mat,i,0,norm);
	}
	cvDiv(e3,norm_mat,e3);

	CvMat *e2 = cvCreateMat( 3, 1, CV_64FC1 );
	cvCrossProduct(e3, e1, e2);
	

	//以正確
//app_XRs.close();

	//fstream app_XRs("XRs.txt",ios::app);
	/*
	TooN::Matrix<3,3> T;
	T[0] = e1;
	T[1] = e2;
	T[2] = e3;

	f3 = T*f3;
	*/

//改到這
	CvMat *T = cvCreateMat( 3, 3, CV_64FC1 );
	for(int i=0;i<3;i++)
	{
		cvmSet(T, 0, i, cvmGet(e1,i,0));
		cvmSet(T, 1, i, cvmGet(e2,i,0));
		cvmSet(T, 2, i, cvmGet(e3,i,0));
	}


	
	/*app_XRs<<"f3"<<endl;
	app_XRs<<cvmGet(f3,0,0)<<"    "<<cvmGet(f3,0,1)<<"    "<<cvmGet(f3,0,2)<<"    "<<endl;
	cvTranspose(f3,f3);
	app_XRs<<"f3tr"<<endl;
	app_XRs<<cvmGet(f3,0,0)<<"    "<<cvmGet(f3,0,1)<<"    "<<cvmGet(f3,0,2)<<"    "<<endl;*/
	//中斷點
	
	cvmMul(T, f3, f3);

	// Reinforce that f3[2] > 0 for having theta in [0;pi]
	/*
	if( f3[2] > 0 )
	{
		f1 = featureVectors.T()[1];
		f2 = featureVectors.T()[0];
		f3 = featureVectors.T()[2];

		e1 = f1;
		e3 = f1 ^ f2;
		e3 = e3 / TooN::norm(e3);
		e2 = e3 ^ e1;

		T[0] = e1;
		T[1] = e2;
		T[2] = e3;

		f3 = T*f3;

		P1 = worldPoints.T()[1];
		P2 = worldPoints.T()[0];
		P3 = worldPoints.T()[2];
	}*/
	if(cvmGet(f3,2,0)>0)
	{
		for(int i=0;i<3;i++)
		{
		cvmSet(f2 , i ,0, worldPoints[i]);
		cvmSet(f1 , i ,0, worldPoints[i+3]);
		cvmSet(f3 , i, 0 ,worldPoints[i+6]);
		}


		e1 = cvCloneMat(f1);
		cvCrossProduct(f1, f2, e3);
		//norm = cvNorm(e3, NULL,CV_L1);
		norm=sqrt(cvmGet(e3,0,0)*cvmGet(e3,0,0)+cvmGet(e3,1,0)*cvmGet(e3,1,0)+cvmGet(e3,2,0)*cvmGet(e3,2,0));



		for(int i=0;i<3;i++)
		{
			cvmSet(norm_mat,i,0,norm);
		}
		cvDiv(e3,norm_mat,e3);
		cvCrossProduct(e3, e1, e2);


		for(int i=0;i<3;i++)
		{
			cvmSet(T, 0, i, cvmGet(e1,i,0));
			cvmSet(T, 1, i, cvmGet(e2,i,0));
			cvmSet(T, 2, i, cvmGet(e3,i,0));
		}
		cvmMul(T, f3, f3);

		for(int i=0;i<3;i++)
		{
		cvmSet(P2 ,i,0, featureVectors[i]);
		cvmSet(P1 ,i,0 ,featureVectors[i+3]);
		cvmSet(P3 ,i,0, featureVectors[i+6]);
		}

	}





	// Creation of intermediate world frame
	/*
	TooN::Vector<3> n1 = P2-P1;
	n1 = n1 / TooN::norm(n1);
	TooN::Vector<3> n3 = n1 ^ (P3-P1);
	n3 = n3 / TooN::norm(n3);
	TooN::Vector<3> n2 = n3 ^ n1;

	TooN::Matrix<3,3> N;
	N[0] = n1;
	N[1] = n2;
	N[2] = n3;
	*/
	//改到這2
	CvMat* n1 = cvCreateMat(3,1,CV_64FC1);
	cvSub(P2,P1,n1);
	//norm = cvNorm(n1, NULL,CV_L1);
	norm=sqrt(cvmGet(n1,0,0)*cvmGet(n1,0,0)+cvmGet(n1,1,0)*cvmGet(n1,1,0)+cvmGet(n1,2,0)*cvmGet(n1,2,0));
	

	for(int i=0;i<3;i++)
	{
		cvmSet(norm_mat,i,0,norm);
	}
	cvDiv(n1,norm_mat,n1);
	
	
	CvMat* n3 = cvCreateMat(3,1,CV_64FC1);	
	cvSub(P3, P1, norm_mat);	
	cvCrossProduct(n1, norm_mat,n3);
	//norm = cvNorm(n3, NULL,CV_L1);
	norm=sqrt(cvmGet(n3,0,0)*cvmGet(n3,0,0)+cvmGet(n3,1,0)*cvmGet(n3,1,0)+cvmGet(n3,2,0)*cvmGet(n3,2,0));

	for(int i=0;i<3;i++)
	{
		cvmSet(norm_mat,i,0,norm);
	}
	cvDiv(n3,norm_mat,n3);

	CvMat* n2 = cvCreateMat(3,1,CV_64FC1);	
	cvCrossProduct(n3,n1, n2);


	CvMat* N = cvCreateMat(3,3,CV_64FC1);
	for(int i=0;i<3;i++)
	{
		cvmSet(N, 0, i, cvmGet(n1,i,0));
		cvmSet(N, 1, i, cvmGet(n2,i,0));
		cvmSet(N, 2, i, cvmGet(n3,i,0));
	}


	// Extraction of known parameters
	/*
	P3 = N*(P3-P1);

	double d_12 = TooN::norm(P2-P1);
	double f_1 = f3[0]/f3[2];
	double f_2 = f3[1]/f3[2];
	double p_1 = P3[0];
	double p_2 = P3[1];

	double cos_beta = f1 * f2;
	double b = 1/(1-pow(cos_beta,2)) - 1;

	if (cos_beta < 0)
		b = -sqrt(b);
	else
		b = sqrt(b);
		*/
	cvSub(P3, P1,norm_mat);	

	cvmMul(N, norm_mat,P3);
	


	cvSub(P2, P1,norm_mat);
	//double d_12 = cvNorm(norm_mat,NULL,CV_L1);
	double d_12 =sqrt(cvmGet(norm_mat,0,0)*cvmGet(norm_mat,0,0)+cvmGet(norm_mat,1,0)*cvmGet(norm_mat,1,0)+cvmGet(norm_mat,2,0)*cvmGet(norm_mat,2,0));
	double f_1  = cvmGet(f3,0,0)/cvmGet(f3,2,0);
	double f_2  = cvmGet(f3,1,0)/cvmGet(f3,2,0);
	double p_1  = cvmGet(P3,0,0);
	double p_2  = cvmGet(P3,1,0);
	double cos_beta = cvDotProduct(f1, f2);
	double b = 1/(1 - cos_beta*cos_beta) - 1;
	
	if (cos_beta < 0)
		b = -sqrt(b);
	else
		b = sqrt(b);

	// Definition of temporary variables for avoiding multiple computation

	double f_1_pw2 = pow(f_1,2);
	double f_2_pw2 = pow(f_2,2);
	double p_1_pw2 = pow(p_1,2);
	double p_1_pw3 = p_1_pw2 * p_1;
	double p_1_pw4 = p_1_pw3 * p_1;
	double p_2_pw2 = pow(p_2,2);
	double p_2_pw3 = p_2_pw2 * p_2;
	double p_2_pw4 = p_2_pw3 * p_2;
	double d_12_pw2 = pow(d_12,2);
	double b_pw2 = pow(b,2);

	// Computation of factors of 4th degree polynomial

	//TooN::Vector<5> factors;
	double factors[5];
	factors[0] = -f_2_pw2*p_2_pw4
				 -p_2_pw4*f_1_pw2
				 -p_2_pw4;

	factors[1] = 2*p_2_pw3*d_12*b
				 +2*f_2_pw2*p_2_pw3*d_12*b
				 -2*f_2*p_2_pw3*f_1*d_12;

	factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2
			     -f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
				 -f_2_pw2*p_2_pw2*d_12_pw2
				 +f_2_pw2*p_2_pw4
				 +p_2_pw4*f_1_pw2
				 +2*p_1*p_2_pw2*d_12
				 +2*f_1*f_2*p_1*p_2_pw2*d_12*b
				 -p_2_pw2*p_1_pw2*f_1_pw2
				 +2*p_1*p_2_pw2*f_2_pw2*d_12
				 -p_2_pw2*d_12_pw2*b_pw2
				 -2*p_1_pw2*p_2_pw2;

	factors[3] = 2*p_1_pw2*p_2*d_12*b
				 +2*f_2*p_2_pw3*f_1*d_12
				 -2*f_2_pw2*p_2_pw3*d_12*b
				 -2*p_1*p_2*d_12_pw2*b;

	factors[4] = -2*f_2*p_2_pw2*f_1*p_1*d_12*b
				 +f_2_pw2*p_2_pw2*d_12_pw2
				 +2*p_1_pw3*d_12
				 -p_1_pw2*d_12_pw2
				 +f_2_pw2*p_2_pw2*p_1_pw2
				 -p_1_pw4
				 -2*f_2_pw2*p_2_pw2*p_1*d_12
				 +p_2_pw2*f_1_pw2*p_1_pw2
				 +f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;
	// Computation of roots

	//TooN::Vector<4> realRoots;
	//this->solveQuartic( factors, realRoots );
	
	double realRoots[4] = {0, 0, 0, 0};
	solve_deg4(factors[0],factors[1],factors[2],factors[3],factors[4],realRoots[0],realRoots[1],realRoots[2],realRoots[3]);
	// Backsubstitution of each solution

	for(int i=0; i<4; i++)
	{
		double cot_alpha = (-f_1*p_1/f_2-realRoots[i]*p_2+d_12*b)/(-f_1*realRoots[i]*p_2/f_2+p_1-d_12);

		double cos_theta = realRoots[i];
		double sin_theta = sqrt(1-pow(realRoots[i],2));
		double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
		double cos_alpha = sqrt(1-pow(sin_alpha,2));

		if (cot_alpha < 0)
			cos_alpha = -cos_alpha;
	
		/*
		TooN::Vector<3> C = TooN::makeVector(
				d_12*cos_alpha*(sin_alpha*b+cos_alpha),
				cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha),
				sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));

		C = P1 + N.T()*C;

		TooN::Matrix<3,3> R;
		R[0] = TooN::makeVector(	-cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta );
		R[1] = TooN::makeVector(	sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta );
		R[2] = TooN::makeVector(	0,				-sin_theta,				cos_theta );

		R = N.T()*R.T()*T;

		solutions.T()[i*4] = C;
		solutions.T()[i*4+1] = R.T()[0];
		solutions.T()[i*4+2] = R.T()[1];
		solutions.T()[i*4+3] = R.T()[2];
		*/
		CvMat* C = cvCreateMat(3,1,CV_64FC1);
		cvmSet( C, 0, 0, d_12*cos_alpha*(sin_alpha*b+cos_alpha));
		cvmSet( C, 1, 0, cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));
		cvmSet( C, 2, 0, sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));
		CvMat* NT = cvCreateMat(3,3,CV_64FC1);
		cvTranspose(N, NT);
		CvMat* NTC = cvCreateMat(3,1,CV_64FC1);
		cvmMul( NT, C, NTC);
		cvAdd( P1, NTC, C);

		Matrix temp_X;

		//轉回map_feature裡視線向量的座標系統
		//temp_X.data_[0] = cvmGet(C,0,0);
		//temp_X.data_[2] = -cvmGet(C,1,0);
		//temp_X.data_[1] = cvmGet(C,2,0);

		//20151221
		//原座標系統(左手)
		temp_X.data_[0] = cvmGet(C, 0, 0);
		temp_X.data_[1] = cvmGet(C, 1, 0);
		temp_X.data_[2] = cvmGet(C, 2, 0);

		X.push_back(temp_X);

		CvMat* R = cvCreateMat(3,3,CV_64FC1);
		cvmSet( R, 0, 0, -cos_alpha);	cvmSet( R, 0, 1, -sin_alpha*cos_theta);			cvmSet( R, 0, 2,	-sin_alpha*sin_theta);
		cvmSet( R, 1, 0,  sin_alpha);	cvmSet( R, 1, 1, -cos_alpha*cos_theta);	cvmSet( R, 1, 2, -cos_alpha*sin_theta);
		cvmSet( R, 2, 0,		  0);	cvmSet( R, 2, 1, -sin_theta);			cvmSet( R, 2, 2,  cos_theta);

		CvMat* RT = cvCreateMat(3,3,CV_64FC1);
		cvTranspose(R, RT);

		CvMat* NTRT = cvCreateMat(3,3,CV_64FC1);
		cvmMul( NT, RT, NTRT);

		cvmMul( NTRT, T, R);

		Rectangular temp_R;
		memset(&temp_R,0,sizeof(Rectangular));
		
		//R12(前一刻看這一刻)
		temp_R.data_[0][0] = cvmGet(R, 0, 0);
		temp_R.data_[0][1] = cvmGet(R, 0, 1);
		temp_R.data_[0][2] = cvmGet(R, 0, 2);
			
		//temp_R.data_[1][0] = cvmGet(R, 2, 0);
		//temp_R.data_[1][1] = cvmGet(R, 2, 1);
		//temp_R.data_[1][2] = cvmGet(R, 2, 2);

		//temp_R.data_[2][0] = -cvmGet(R, 1, 0);
		//temp_R.data_[2][1] = -cvmGet(R, 1, 1);
		//temp_R.data_[2][2] = -cvmGet(R, 1, 2);

		//20151127改
		temp_R.data_[1][0] = cvmGet(R, 1, 0);
		temp_R.data_[1][1] = cvmGet(R, 1, 1);
		temp_R.data_[1][2] = cvmGet(R, 1, 2);

		temp_R.data_[2][0] = cvmGet(R, 2, 0);
		temp_R.data_[2][1] = cvmGet(R, 2, 1);
		temp_R.data_[2][2] = cvmGet(R, 2, 2);

		Rs.push_back(temp_R);

		//20151103黃聖凱加
		cvReleaseMat(&R);
		cvReleaseMat(&RT);
		cvReleaseMat(&NTRT);
		cvReleaseMat(&C);
		cvReleaseMat(&NT);
		cvReleaseMat(&NTC);
	}
	cvReleaseMat(&P1);
	cvReleaseMat(&P2);
	cvReleaseMat(&P3);
	cvReleaseMat(&N);
	cvReleaseMat(&n1);
	cvReleaseMat(&n2);
	cvReleaseMat(&n3);
	cvReleaseMat(&norm_mat);
	cvReleaseMat(&temp1);
	cvReleaseMat(&temp2);
	cvReleaseMat(&temp3);
	cvReleaseMat(&T);
	cvReleaseMat(&e1);
	cvReleaseMat(&e2);
	cvReleaseMat(&e3);
	cvReleaseMat(&f1);
	cvReleaseMat(&f2);
	cvReleaseMat(&f3);
	return 0;
}

int P3PSolver::computePoses2(double*featureVectors, double*worldPoints, vector<Matrix>&X, vector<Rectangular>& Rs)
{



	// Extraction of world points
	/*
	TooN::Vector<3> P1 = worldPoints.T()[0];
	TooN::Vector<3> P2 = worldPoints.T()[1];
	TooN::Vector<3> P3 = worldPoints.T()[2];
	*/

	CvMat *P1 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *P2 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *P3 = cvCreateMat(3, 1, CV_64FC1);

	for (int i = 0; i<3; i++)	//前一刻視線向量
	{
		cvmSet(P1, i, 0, featureVectors[i]);
		cvmSet(P2, i, 0, featureVectors[i + 3]);
		cvmSet(P3, i, 0, featureVectors[i + 6]);
	}


	// Verification that world points are not colinear
	/*
	TooN::Vector<3> temp1 = P2 - P1;
	TooN::Vector<3> temp2 = P3 - P1;
	if(TooN::norm(temp1 ^ temp2) == 0)
	return -1;
	*/
	CvMat *temp1 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *temp2 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *temp3 = cvCreateMat(3, 1, CV_64FC1);
	cvSub(P2, P1, temp1);
	cvSub(P3, P1, temp2);

	//cvmMul(temp1, temp2, temp3);
	//if(cvNorm(temp3,NULL,CV_L1)==0)return -1;

	// Extraction of feature vectors
	/*
	TooN::Vector<3> f1 = featureVectors.T()[0];
	TooN::Vector<3> f2 = featureVectors.T()[1];
	TooN::Vector<3> f3 = featureVectors.T()[2];
	*/
	CvMat *f1 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *f2 = cvCreateMat(3, 1, CV_64FC1);
	CvMat *f3 = cvCreateMat(3, 1, CV_64FC1);


	for (int i = 0; i<3; i++)	//這一刻單位視線向量
	{
		cvmSet(f1, i, 0, worldPoints[i]);
		cvmSet(f2, i, 0, worldPoints[i + 3]);
		cvmSet(f3, i, 0, worldPoints[i + 6]);
	}

	// Creation of intermediate camera frame
	/*
	TooN::Vector<3> e1 = f1;
	TooN::Vector<3> e3 = f1 ^ f2;
	e3 = e3 / TooN::norm(e3);
	TooN::Vector<3> e2 = e3 ^ e1;
	*/
	CvMat *e1 = cvCloneMat(f1);
	CvMat *e3 = cvCreateMat(3, 1, CV_64FC1);

	cvCrossProduct(f1, f2, e3);



	//double norm = cvNorm(e3, NULL,CV_L1);
	double norm = sqrt(cvmGet(e3, 0, 0)*cvmGet(e3, 0, 0) + cvmGet(e3, 1, 0)*cvmGet(e3, 1, 0) + cvmGet(e3, 2, 0)*cvmGet(e3, 2, 0));


	CvMat *norm_mat = cvCreateMat(3, 1, CV_64FC1);
	for (int i = 0; i<3; i++)
	{
		cvmSet(norm_mat, i, 0, norm);
	}
	cvDiv(e3, norm_mat, e3);

	CvMat *e2 = cvCreateMat(3, 1, CV_64FC1);
	cvCrossProduct(e3, e1, e2);


	//以正確
	//app_XRs.close();

	//fstream app_XRs("XRs.txt",ios::app);
	/*
	TooN::Matrix<3,3> T;
	T[0] = e1;
	T[1] = e2;
	T[2] = e3;

	f3 = T*f3;
	*/

	//改到這
	CvMat *T = cvCreateMat(3, 3, CV_64FC1);
	for (int i = 0; i<3; i++)
	{
		cvmSet(T, 0, i, cvmGet(e1, i, 0));
		cvmSet(T, 1, i, cvmGet(e2, i, 0));
		cvmSet(T, 2, i, cvmGet(e3, i, 0));
	}



	/*app_XRs<<"f3"<<endl;
	app_XRs<<cvmGet(f3,0,0)<<"    "<<cvmGet(f3,0,1)<<"    "<<cvmGet(f3,0,2)<<"    "<<endl;
	cvTranspose(f3,f3);
	app_XRs<<"f3tr"<<endl;
	app_XRs<<cvmGet(f3,0,0)<<"    "<<cvmGet(f3,0,1)<<"    "<<cvmGet(f3,0,2)<<"    "<<endl;*/
	//中斷點

	cvmMul(T, f3, f3);

	// Reinforce that f3[2] > 0 for having theta in [0;pi]
	/*
	if( f3[2] > 0 )
	{
	f1 = featureVectors.T()[1];
	f2 = featureVectors.T()[0];
	f3 = featureVectors.T()[2];

	e1 = f1;
	e3 = f1 ^ f2;
	e3 = e3 / TooN::norm(e3);
	e2 = e3 ^ e1;

	T[0] = e1;
	T[1] = e2;
	T[2] = e3;

	f3 = T*f3;

	P1 = worldPoints.T()[1];
	P2 = worldPoints.T()[0];
	P3 = worldPoints.T()[2];
	}*/
	if (cvmGet(f3, 2, 0)>0)
	{
		for (int i = 0; i<3; i++)
		{
			cvmSet(f2, i, 0, worldPoints[i]);
			cvmSet(f1, i, 0, worldPoints[i + 3]);
			cvmSet(f3, i, 0, worldPoints[i + 6]);
		}


		e1 = cvCloneMat(f1);
		cvCrossProduct(f1, f2, e3);
		//norm = cvNorm(e3, NULL,CV_L1);
		norm = sqrt(cvmGet(e3, 0, 0)*cvmGet(e3, 0, 0) + cvmGet(e3, 1, 0)*cvmGet(e3, 1, 0) + cvmGet(e3, 2, 0)*cvmGet(e3, 2, 0));



		for (int i = 0; i<3; i++)
		{
			cvmSet(norm_mat, i, 0, norm);
		}
		cvDiv(e3, norm_mat, e3);
		cvCrossProduct(e3, e1, e2);


		for (int i = 0; i<3; i++)
		{
			cvmSet(T, 0, i, cvmGet(e1, i, 0));
			cvmSet(T, 1, i, cvmGet(e2, i, 0));
			cvmSet(T, 2, i, cvmGet(e3, i, 0));
		}
		cvmMul(T, f3, f3);

		for (int i = 0; i<3; i++)
		{
			cvmSet(P2, i, 0, featureVectors[i]);
			cvmSet(P1, i, 0, featureVectors[i + 3]);
			cvmSet(P3, i, 0, featureVectors[i + 6]);
		}

	}





	// Creation of intermediate world frame
	/*
	TooN::Vector<3> n1 = P2-P1;
	n1 = n1 / TooN::norm(n1);
	TooN::Vector<3> n3 = n1 ^ (P3-P1);
	n3 = n3 / TooN::norm(n3);
	TooN::Vector<3> n2 = n3 ^ n1;

	TooN::Matrix<3,3> N;
	N[0] = n1;
	N[1] = n2;
	N[2] = n3;
	*/
	//改到這2
	CvMat* n1 = cvCreateMat(3, 1, CV_64FC1);
	cvSub(P2, P1, n1);
	//norm = cvNorm(n1, NULL,CV_L1);
	norm = sqrt(cvmGet(n1, 0, 0)*cvmGet(n1, 0, 0) + cvmGet(n1, 1, 0)*cvmGet(n1, 1, 0) + cvmGet(n1, 2, 0)*cvmGet(n1, 2, 0));


	for (int i = 0; i<3; i++)
	{
		cvmSet(norm_mat, i, 0, norm);
	}
	cvDiv(n1, norm_mat, n1);


	CvMat* n3 = cvCreateMat(3, 1, CV_64FC1);
	cvSub(P3, P1, norm_mat);
	cvCrossProduct(n1, norm_mat, n3);
	//norm = cvNorm(n3, NULL,CV_L1);
	norm = sqrt(cvmGet(n3, 0, 0)*cvmGet(n3, 0, 0) + cvmGet(n3, 1, 0)*cvmGet(n3, 1, 0) + cvmGet(n3, 2, 0)*cvmGet(n3, 2, 0));

	for (int i = 0; i<3; i++)
	{
		cvmSet(norm_mat, i, 0, norm);
	}
	cvDiv(n3, norm_mat, n3);

	CvMat* n2 = cvCreateMat(3, 1, CV_64FC1);
	cvCrossProduct(n3, n1, n2);


	CvMat* N = cvCreateMat(3, 3, CV_64FC1);
	for (int i = 0; i<3; i++)
	{
		cvmSet(N, 0, i, cvmGet(n1, i, 0));
		cvmSet(N, 1, i, cvmGet(n2, i, 0));
		cvmSet(N, 2, i, cvmGet(n3, i, 0));
	}


	// Extraction of known parameters
	/*
	P3 = N*(P3-P1);

	double d_12 = TooN::norm(P2-P1);
	double f_1 = f3[0]/f3[2];
	double f_2 = f3[1]/f3[2];
	double p_1 = P3[0];
	double p_2 = P3[1];

	double cos_beta = f1 * f2;
	double b = 1/(1-pow(cos_beta,2)) - 1;

	if (cos_beta < 0)
	b = -sqrt(b);
	else
	b = sqrt(b);
	*/
	cvSub(P3, P1, norm_mat);

	cvmMul(N, norm_mat, P3);



	cvSub(P2, P1, norm_mat);
	//double d_12 = cvNorm(norm_mat,NULL,CV_L1);
	double d_12 = sqrt(cvmGet(norm_mat, 0, 0)*cvmGet(norm_mat, 0, 0) + cvmGet(norm_mat, 1, 0)*cvmGet(norm_mat, 1, 0) + cvmGet(norm_mat, 2, 0)*cvmGet(norm_mat, 2, 0));
	double f_1 = cvmGet(f3, 0, 0) / cvmGet(f3, 2, 0);
	double f_2 = cvmGet(f3, 1, 0) / cvmGet(f3, 2, 0);
	double p_1 = cvmGet(P3, 0, 0);
	double p_2 = cvmGet(P3, 1, 0);
	double cos_beta = cvDotProduct(f1, f2);
	double b = 1 / (1 - cos_beta*cos_beta) - 1;

	if (cos_beta < 0)
		b = -sqrt(b);
	else
		b = sqrt(b);

	// Definition of temporary variables for avoiding multiple computation

	double f_1_pw2 = pow(f_1, 2);
	double f_2_pw2 = pow(f_2, 2);
	double p_1_pw2 = pow(p_1, 2);
	double p_1_pw3 = p_1_pw2 * p_1;
	double p_1_pw4 = p_1_pw3 * p_1;
	double p_2_pw2 = pow(p_2, 2);
	double p_2_pw3 = p_2_pw2 * p_2;
	double p_2_pw4 = p_2_pw3 * p_2;
	double d_12_pw2 = pow(d_12, 2);
	double b_pw2 = pow(b, 2);

	// Computation of factors of 4th degree polynomial

	//TooN::Vector<5> factors;
	double factors[5];
	factors[0] = -f_2_pw2*p_2_pw4
		- p_2_pw4*f_1_pw2
		- p_2_pw4;

	factors[1] = 2 * p_2_pw3*d_12*b
		+ 2 * f_2_pw2*p_2_pw3*d_12*b
		- 2 * f_2*p_2_pw3*f_1*d_12;

	factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2
		- f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
		- f_2_pw2*p_2_pw2*d_12_pw2
		+ f_2_pw2*p_2_pw4
		+ p_2_pw4*f_1_pw2
		+ 2 * p_1*p_2_pw2*d_12
		+ 2 * f_1*f_2*p_1*p_2_pw2*d_12*b
		- p_2_pw2*p_1_pw2*f_1_pw2
		+ 2 * p_1*p_2_pw2*f_2_pw2*d_12
		- p_2_pw2*d_12_pw2*b_pw2
		- 2 * p_1_pw2*p_2_pw2;

	factors[3] = 2 * p_1_pw2*p_2*d_12*b
		+ 2 * f_2*p_2_pw3*f_1*d_12
		- 2 * f_2_pw2*p_2_pw3*d_12*b
		- 2 * p_1*p_2*d_12_pw2*b;

	factors[4] = -2 * f_2*p_2_pw2*f_1*p_1*d_12*b
		+ f_2_pw2*p_2_pw2*d_12_pw2
		+ 2 * p_1_pw3*d_12
		- p_1_pw2*d_12_pw2
		+ f_2_pw2*p_2_pw2*p_1_pw2
		- p_1_pw4
		- 2 * f_2_pw2*p_2_pw2*p_1*d_12
		+ p_2_pw2*f_1_pw2*p_1_pw2
		+ f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;
	// Computation of roots

	//TooN::Vector<4> realRoots;
	//this->solveQuartic( factors, realRoots );

	double realRoots[4] = { 0, 0, 0, 0 };
	solve_deg4(factors[0], factors[1], factors[2], factors[3], factors[4], realRoots[0], realRoots[1], realRoots[2], realRoots[3]);
	// Backsubstitution of each solution

	for (int i = 0; i<4; i++)
	{
		double cot_alpha = (-f_1*p_1 / f_2 - realRoots[i] * p_2 + d_12*b) / (-f_1*realRoots[i] * p_2 / f_2 + p_1 - d_12);

		double cos_theta = realRoots[i];
		double sin_theta = sqrt(1 - pow(realRoots[i], 2));
		double sin_alpha = sqrt(1 / (pow(cot_alpha, 2) + 1));
		double cos_alpha = sqrt(1 - pow(sin_alpha, 2));

		if (cot_alpha < 0)
			cos_alpha = -cos_alpha;

		/*
		TooN::Vector<3> C = TooN::makeVector(
		d_12*cos_alpha*(sin_alpha*b+cos_alpha),
		cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha),
		sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));

		C = P1 + N.T()*C;

		TooN::Matrix<3,3> R;
		R[0] = TooN::makeVector(	-cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta );
		R[1] = TooN::makeVector(	sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta );
		R[2] = TooN::makeVector(	0,				-sin_theta,				cos_theta );

		R = N.T()*R.T()*T;

		solutions.T()[i*4] = C;
		solutions.T()[i*4+1] = R.T()[0];
		solutions.T()[i*4+2] = R.T()[1];
		solutions.T()[i*4+3] = R.T()[2];
		*/
		CvMat* C = cvCreateMat(3, 1, CV_64FC1);
		cvmSet(C, 0, 0, d_12*cos_alpha*(sin_alpha*b + cos_alpha));
		cvmSet(C, 1, 0, cos_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha));
		cvmSet(C, 2, 0, sin_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha));
		CvMat* NT = cvCreateMat(3, 3, CV_64FC1);
		cvTranspose(N, NT);
		CvMat* NTC = cvCreateMat(3, 1, CV_64FC1);
		cvmMul(NT, C, NTC);
		cvAdd(P1, NTC, C);

		Matrix temp_X;

		//轉回map_feature裡視線向量的座標系統
		temp_X.data_[0] = cvmGet(C,0,0);
		temp_X.data_[2] = -cvmGet(C,1,0);
		temp_X.data_[1] = cvmGet(C,2,0);

		//20151221
		//原座標系統(左手)
		//temp_X.data_[0] = cvmGet(C, 0, 0);
		//temp_X.data_[1] = cvmGet(C, 1, 0);
		//temp_X.data_[2] = cvmGet(C, 2, 0);

		X.push_back(temp_X);

		CvMat* R = cvCreateMat(3, 3, CV_64FC1);
		cvmSet(R, 0, 0, -cos_alpha);	cvmSet(R, 0, 1, -sin_alpha*cos_theta);			cvmSet(R, 0, 2, -sin_alpha*sin_theta);
		cvmSet(R, 1, 0, sin_alpha);	cvmSet(R, 1, 1, -cos_alpha*cos_theta);	cvmSet(R, 1, 2, -cos_alpha*sin_theta);
		cvmSet(R, 2, 0, 0);	cvmSet(R, 2, 1, -sin_theta);			cvmSet(R, 2, 2, cos_theta);

		CvMat* RT = cvCreateMat(3, 3, CV_64FC1);
		cvTranspose(R, RT);

		CvMat* NTRT = cvCreateMat(3, 3, CV_64FC1);
		cvmMul(NT, RT, NTRT);

		cvmMul(NTRT, T, R);

		Rectangular temp_R;
		memset(&temp_R, 0, sizeof(Rectangular));

		//R12(前一刻看這一刻)
		temp_R.data_[0][0] = cvmGet(R, 0, 0);
		temp_R.data_[0][1] = cvmGet(R, 0, 1);
		temp_R.data_[0][2] = cvmGet(R, 0, 2);

		temp_R.data_[1][0] = cvmGet(R, 2, 0);
		temp_R.data_[1][1] = cvmGet(R, 2, 1);
		temp_R.data_[1][2] = cvmGet(R, 2, 2);

		temp_R.data_[2][0] = -cvmGet(R, 1, 0);
		temp_R.data_[2][1] = -cvmGet(R, 1, 1);
		temp_R.data_[2][2] = -cvmGet(R, 1, 2);

		//20151127改
		//temp_R.data_[1][0] = cvmGet(R, 1, 0);
		//temp_R.data_[1][1] = cvmGet(R, 1, 1);
		//temp_R.data_[1][2] = cvmGet(R, 1, 2);

		//temp_R.data_[2][0] = cvmGet(R, 2, 0);
		//temp_R.data_[2][1] = cvmGet(R, 2, 1);
		//temp_R.data_[2][2] = cvmGet(R, 2, 2);

		Rs.push_back(temp_R);

		//20151103黃聖凱加
		cvReleaseMat(&R);
		cvReleaseMat(&RT);
		cvReleaseMat(&NTRT);
		cvReleaseMat(&C);
		cvReleaseMat(&NT);
		cvReleaseMat(&NTC);
	}
	cvReleaseMat(&P1);
	cvReleaseMat(&P2);
	cvReleaseMat(&P3);
	cvReleaseMat(&N);
	cvReleaseMat(&n1);
	cvReleaseMat(&n2);
	cvReleaseMat(&n3);
	cvReleaseMat(&norm_mat);
	cvReleaseMat(&temp1);
	cvReleaseMat(&temp2);
	cvReleaseMat(&temp3);
	cvReleaseMat(&T);
	cvReleaseMat(&e1);
	cvReleaseMat(&e2);
	cvReleaseMat(&e3);
	cvReleaseMat(&f1);
	cvReleaseMat(&f2);
	cvReleaseMat(&f3);
	return 0;
}