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

#include "polynom_solver.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>

#include <cv.h>
using namespace std;


//Our zero
#define Epsilon 1.0e-5

class IxIy
{
public:
	double data[2];
};

class Matrix
{
public:
	  double data_[3];
};

class root
{
public:
	  double data_[5];
};

class Rectangular
{
public:
	  double data_[3][3];
};

class P3PSolver
{
public:
	P3PSolver();
	virtual ~P3PSolver();

	void SetIntrinsic(double f_u,double f_v,double u_0,double v_0,double k_1,double k_2,double p_1,double p_2);
	void SetIntrinsic(double *param);
	void SetPointsCorrespondance(double *ptw,double *pti);
	void Solve(vector<Matrix>& X , vector< Rectangular >& Rs , vector< Matrix >& ts);
	void Initial_RANSAC(void);
	int P3PSolver::computePoses( double* featureVectors,double* worldPoints, vector<Matrix>&X, vector<Rectangular>& Rs );
	int P3PSolver::computePoses2(double* featureVectors, double* worldPoints, vector<Matrix>&X, vector<Rectangular>& Rs);
private:
	void GetCoeffcientofP3P(double Rab, double Rac, double Rbc,  
		double Cab, double Cac, double Cbc,
		double &G4, double &G3, double &G2, double &G1, double &G0);

public:
	
	//Roots of P3P
	//Format: [[a0 b0 c0 x0 y0],[a1 b1 c1 x1 y1]...]
	double fu,fv,u0,v0;

	//bool yes_no;
private:	
	
	void MatrixCopy( CvMat *mat, CvMat *submat, const int x, const int y, const int subx, const int suby, const int width, const int height );
	void Norm(CvMat* A , CvMat *B);

	//World & Image coordinates
	CvMat *W;
	CvMat *I;

	//Intrinsic parameters


};
