#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter()
{
	
}



// 狀態 X 預測
void KalmanFilter::State_Predict( CvMat* X, const CvMat* A )
{

	cvmMul( A, X, X ); // X = A * X

}

// 狀態 X 預測
void KalmanFilter::State_Predict( CvMat* X, const CvMat* A, const CvMat* B, const CvMat* u )
{

	CvMat* Bu = cvCreateMat( B->rows, u->cols, CV_64FC1 );
	cvmMul( B, u, Bu ); // Bu = B * u
	cvGEMM( A, X, 1, Bu, 1, X ); // X = (A * X * 1) + (Bu * 1)


	cvReleaseMat(&Bu);
}


// 狀態共變異數 P 預測
void KalmanFilter::State_Covariance_Predict( CvMat* P, const CvMat* A, const CvMat* WQWt )
{

	CvMat* AP = cvCreateMat( A->rows, P->cols, CV_64FC1 );
	cvmMul( A, P, AP ); // AP = A * P

	CvMat* At = cvCreateMat( A->cols, A->rows, CV_64FC1 );
	cvTranspose( A, At ); // At = A轉置
	
	cvGEMM( AP, At, 1, WQWt, 1, P ); // P = (AP * At * 1) + (WQWt * 1)


	cvReleaseMat(&AP);
	cvReleaseMat(&At);

}

// 創新量共變異數 S
void KalmanFilter::Innovation_Covariance( CvMat* S, const CvMat* P, const CvMat* H, const CvMat* VRVt )
{

	CvMat* HP = cvCreateMat( H->rows, P->cols, CV_64FC1 );	
	cvmMul( H, P, HP ); // HP = H * P

	CvMat* Ht = cvCreateMat( H->cols, H->rows, CV_64FC1 );
	cvTranspose( H, Ht ); // Ht = H轉置
	
	cvGEMM( HP, Ht, 1, VRVt, 1, S ); // S = (HP * Ht * 1) + (VRVt * 1)


	cvReleaseMat(&HP);
	cvReleaseMat(&Ht);

}


// 卡爾曼增益 K
void KalmanFilter::Kalman_Gain( CvMat* K, const CvMat* H, const CvMat* S, const CvMat* P )
{

	CvMat* Ht = cvCreateMat( H->cols, H->rows, CV_64FC1 );
	cvTranspose( H, Ht ); // Ht = H轉置

	CvMat* PHt = cvCreateMat( P->rows, Ht->cols, CV_64FC1 );
	cvmMul( P, Ht, PHt ); // PHt = P * Ht

	CvMat* inv_S = cvCreateMat( S->rows, S->cols, CV_64FC1 );
	cvInvert( S, inv_S ); // inv_s = S反矩陣

	cvmMul( PHt, inv_S, K ); // K = PHt * inv_S


	cvReleaseMat(&Ht);
	cvReleaseMat(&PHt);
	cvReleaseMat(&inv_S);

}


// 狀態 X 更新
void KalmanFilter::State_Correct( CvMat* X, const CvMat* z, const CvMat* z_pre , const CvMat* K)
{

	CvMat* z_z_pre = cvCreateMat( z->rows, z->cols, CV_64FC1 );
	cvSub( z, z_pre, z_z_pre ); // z_z_pre = z - z_pre

	cvGEMM( K, z_z_pre, 1, X, 1, X ); // X = (K * z_z_pre * 1) + (X * 1)


	cvReleaseMat(&z_z_pre);

}


// 狀態共變異數 P 更新
void KalmanFilter::State_Covariance_Correct( CvMat* P, const CvMat* H, const CvMat* K )
{

	CvMat* HP = cvCreateMat( H->rows, P->cols, CV_64FC1 );	
	cvmMul( H, P, HP ); // HP = H * P

	cvGEMM( K, HP, -1,  P, 1, P ); // P = (K * HP * -1) + (P * 1)


	cvReleaseMat(&HP);

}