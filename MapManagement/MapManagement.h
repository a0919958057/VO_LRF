#include <cv.h>
#include <vector>
#include <fstream>
#include "..//SURF//SURF.h"
using namespace std;



#ifndef _MapManagement_h_
#define _MapManagement_h_











class SingleFeature
{
public:

	double	ix;
	double 	iy;
	int		laplacian; // trace
	int		size; // scale
	float	dir; // 特徵方向
	float	hessian; // 行列式值
	float	descriptor[128]; // 描述向量
	
	bool    on_map;
	
};



class PairFeature
{
public:
	
	int		num;

	double 	ix;
	double 	iy;
	int		laplacian; // trace
	int		size; // scale
	float	dir; // 特徵方向
	float	hessian; // 行列式值
	float	descriptor[128]; // 描述向量

	double 	r_ix;
	double 	r_iy;

	bool    on_map;
	bool    match;
	bool    is_close;
	bool    is_similar;
	
};


class MapFeature
{
public:

	int		num; // 特徵點地圖編號

	double	ix;
	double 	iy;
	double	ix_pre;
	double 	iy_pre;
	int		laplacian; // trace
	int		size; // scale
	float	dir; // 特徵方向
	float	hessian; // 行列式值
	float	original_descriptor[128]; // 原始的描述向量
	float	current_descriptor[128]; // 不斷更新的描述向量

	int     search_window_size;

	double	r_ix;
	double	r_iy;
	
	bool    on_image;
	bool    new_add;
	bool    old_match;
	bool	at_front;
	bool    is_near;
	int		run_times;
	int		detect_times;
	bool	match_stable;

};









class MapManagement
{
public:

	MapManagement();
	virtual ~MapManagement();

	ClassSurf surf01;

	void Initial_MapManagement(void);	
	void GetFeatureData( const IplImage* image_gray, const CvSURFParams params, vector<SingleFeature>& feature );
	void Image_Correction( double& ix, double &iy, const double fu, const double fv, const double u0, const double v0, const double coefficient[8] );
	void Erase_Bad_Feature( vector<MapFeature>& map_feature, vector<int>& erase_map_feature , int& on_image_num, const int max_on_image_num, const int erase_times, const double erase_ratio );
	void Add_New_Feature( vector<SingleFeature> feature, vector<PairFeature>& pair_feature, vector<MapFeature>& map_feature, vector<int>& add_map_feature, int& on_image_num, const int max_on_image_num, const int add_window_size, const double similar_threshold, const int extended );
	


private:

	
	double l_u0;
	double l_fu;
	double r_u0;
	double r_fu;	
	double L;
	double minimum_distance;
	double maximum_distance;


	
		int num;

};


#endif