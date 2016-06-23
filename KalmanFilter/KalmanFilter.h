#include <cv.h>


#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

class KalmanFilter
{

public:

	KalmanFilter();
	virtual ~KalmanFilter();

	void State_Predict( CvMat* X, const CvMat* A );
	void State_Predict( CvMat* X, const CvMat* A, const CvMat* B, const CvMat* u );
	void State_Covariance_Predict( CvMat* P, const CvMat* A, const CvMat* WQWt );
	void Innovation_Covariance( CvMat* S, const CvMat* P, const CvMat* H, const CvMat* VRVt );
	void Kalman_Gain( CvMat* K, const CvMat* H, const CvMat* S, const CvMat* P );
	void State_Correct( CvMat* X, const CvMat* z, const CvMat* z_pre , const CvMat* K);
	void State_Covariance_Correct( CvMat* P, const CvMat* H, const CvMat* K );

};

#endif