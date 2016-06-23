#include "..//MapManagement//MapManagement.h"
#include "..//DataAssociation//DataAssociation.h"
#include "..//KalmanFilter//KalmanFilter.h"

//20151119�s�W�ק�
//#include "..//RANSAC//P3PSolver.h"	//����RANSAC.h�̭�
#include "..//RANSAC//RANSAC.h"

//#include "..//p3p//P3PSolver.h"
//#include <fstream>	//����RANSAC.h�̭�
#include <iomanip>
//#define PI 3.1415926	//����RANSAC.h�̭�

#ifndef _SLAM_h_
#define _SLAM_h_

//20151119����RANSAC.h�̭�
//class Ransac_Pos
//{
//public:
//	vector<int> match_set;
//	double X[3];
//	double Rs[3][3];
//};

//20160127����RANSAC.h�̭�
//class P3dx_Pos
//{
//public:
//	double x;
//	double y;
//	double z;
//	double x_dir;
//	double y_dir;
//	double z_dir;
//
//	double Rwc[3][3];	//20160127
//};

//20151119����RANSAC.h�̭�
//class Keep_Feature
//{
//public:
//	int		num; // �S�x�I�a�Ͻs��
//
//	double	hx;
//	double	hy;
//	double	hz;
//
//	double	l_ix;
//	double 	l_iy;
//	double	r_ix;
//	double 	r_iy;
//
//	int		laplacian; // trace
//	int		size; // scale
//	float	dir; // �S�x��V
//	float	hessian; // ��C����
//	float	original_descriptor[128]; // ��l���y�z�V�q
//	float	current_descriptor[128]; // ���_��s���y�z�V�q
//
//	int     search_window_size;
//	bool    match;
//
//	int times;  //����
//	int stable; //í�w����
//	//int unstable;
//	int count; //�ˬd����
//	int filter;
//	int appear;//�X�{
//
//};


class SLAM
{
public:

	SLAM();
	virtual ~SLAM();

	CvMat *R_matrix;
	CvMat *T_matrix;

	vector<Keep_Feature> map_key_feature;	//20160130
	vector<Keep_Feature> map_feature;
	vector<Keep_Feature> last_map_feature;
	vector<Ransac_Pos> ransac_match;
	vector<P3dx_Pos> p3dxpos;
	vector<CvPoint2D64f> read_Laser_data;
	P3dx_Pos p3dx_x;

	int on_image_num;
	int location[3];

	void Initial_SLAM(void);
	void Run_BinocularSLAM(const IplImage* l_image, const IplImage* r_image, const double sample_time);

	int EKF_Predict_Time;
	int DataAssociation_Time;
	int EKF_Correct_Time;
	int Erase_Bad_Feature_Time;
	int EKF_Diminish_State_Time;
	int Add_New_Feature_Time;
	int EKF_Augment_State_Time;
	int Set_Hessian_Threshold_Time;
	int Match_Time;
	int ransac_time;

	int hessian;
	bool found;
	bool lost;
	bool firstmove;
	bool firststop;
	bool read_map_mode = false;
	bool Online_Laser_Localization_enable = false;
	bool Offline_Laser_Localization_enable = false;
	double Laser_Data_buffer[30000][3];   //Ū���p�g���Z�����

	int PhotoCount;
	int feature_num;
	int stoptime;

private:

	int extended;
	int hessian_threshold;
	int octaves;
	int octave_layers;
	int hessian_error;
	int minimum_hessian_threshold;
	int maximum_hessian_threshold;
	int minimum_Search_Window_Size;
	int maximum_Search_Window_Size;
	int add_window_size;
	double bino_d_match_threshold;
	double original_d_match_threshold;
	double current_d_match_threshold;
	double absolute_d_match_threshold;
	double similar_threshold;
	int erase_times;
	double erase_ratio;
	int max_on_image_num;
	int Map_Covariance_Predict;
	double maximum_distance;
	double Initial_State_Covariance;
	double Sigma_wv;
	double Sigma_ww;
	double Sigma_v;

	double l_u0;
	double l_v0;
	double l_fu;
	double l_fv;
	double l_coefficient[8];
	double r_u0;
	double r_v0;
	double r_fu;
	double r_fv;
	double r_coefficient[8];
	double L;

	MapManagement map;
	KalmanFilter kf;
	DataAssociation datamatch;

	//20151119�[
	RANSAC ransac;

	int count;
	int hessian_threshold_add;
	int hessian_threshold_subtract;

	void R_RANSAC(void);	//20151119����RANSAC.cpp
	void Find_Feature(const IplImage* l_image_gray, const IplImage* r_image_gray, vector<SingleFeature>& l_feature, vector<SingleFeature>& r_feature);
	void Save_Feature(vector<PairFeature>&pair_feature);
	void Comparison_Feature_original_descriptor(vector<PairFeature> &pair_feature);
	void Comparison_Feature_last_map_feature_descriptor(vector<PairFeature> &pair_feature);
	void Comparison_Feature(vector<SingleFeature>&l_feature, vector<SingleFeature>&r_feature, vector<PairFeature>&pair_feature);
	void SLAM_Set_Hessian_Threshold(void);
	void Laser_location();
};


#endif