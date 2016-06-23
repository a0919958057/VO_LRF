#include <highgui.h>
#include <cv.h>
#include <cxcore.h>

#ifndef _SURF_h_
#define _SURF_h_

struct CvSurfHF
{
    int p0, p1, p2, p3;
    float w;
};

class ClassSurf
{
private:
	
	int icvInterpolateKeypoint( float N9[3][9], int dx, int dy, int ds, CvSURFPoint *point );
	void icvResizeHaarPattern( const int src[][5], CvSurfHF* dst, int n, int oldSize, int newSize, int widthStep );
	float icvCalcHaarPattern( const int* origin, const CvSurfHF* f, int n );
	CvSeq* icvFastHessianDetector( const CvMat* sum, const CvMat* mask_sum, CvMemStorage* storage, const CvSURFParams* params );
	
	static int HAAR_SIZE0;
	static int HAAR_SIZE_INC;

public:

	void cvExtractSURF_cv200( const CvArr* _img, const CvArr* _mask, CvSeq** _keypoints, CvSeq** _descriptors, CvMemStorage* storage, CvSURFParams params, int useProvidedKeyPts );
	void cvExtractSURF_cv200_16D( const CvArr* _img, const CvArr* _mask, CvSeq** _keypoints, CvSeq** _descriptors, CvMemStorage* storage, CvSURFParams params, int useProvidedKeyPts );
	void cvExtractSURF_cv200_avg( const CvArr* _img, const CvArr* _mask, CvSeq** _keypoints, CvSeq** _descriptors, CvMemStorage* storage, CvSURFParams params, int useProvidedKeyPts );
	void cvExtractSURF_cv200_16D_avg( const CvArr* _img, const CvArr* _mask, CvSeq** _keypoints, CvSeq** _descriptors, CvMemStorage* storage, CvSURFParams params, int useProvidedKeyPts );



};

#endif