//20151118�s�W
#include "P3PSolver.h"
#include <fstream>
#define PI 3.1415926

class World_Coordinates
{
public:
	int photonum;
	double	X;
	double	Y;
	double	Z;

	double dist;	//20160124�[
	bool accurate;	//20160124�[
};

class Ransac_Pos
{
public:
	int model[3][2];	//20151222�[

	double count_time;	//20151222�[
	int count_P3P_times;	//2015124

	vector<int> match_set;

	double X[3];
	double X_dist;	//20151222�[

	double Rwc[3][3];	//20160127
	double Rs[3][3];
};

class P3dx_Pos
{
public:
	double x;
	double y;
	double z;
	double x_dir;
	double y_dir;
	double z_dir;

	double Rwc[3][3];	//20160127
};

class Keep_Feature
{
public:
	int		num; // �S�x�I�a�Ͻs��

	//20151222�[
	vector<World_Coordinates>Xw;	//�����@�ɮy��
	double deviation;	// �зǮt

	//20151118�[�@�ɮy��
	double	X;
	double	Y;
	double	Z;

	//bool    update;	//�n���n��s

	//20151028
	int ex_photonum;

	//20160127
	double	ex_hx;
	double	ex_hy;
	double	ex_hz;

	double	hx;
	double	hy;
	double	hz;

	double	l_ix;
	double 	l_iy;
	double	r_ix;
	double 	r_iy;

	int		laplacian; // trace
	int		size; // scale
	float	dir; // �S�x��V
	float	hessian; // ��C����
	float	average_descriptor[128]; // ��l���y�z�V�q
	float	current_descriptor[128]; // ���_��s���y�z�V�q

	int     search_window_size;
	bool    match;

	int times;  //����
	int stable; //í�w����
	//int unstable;
	int unstable_count; //�ˬd����
	int filter;
	int appear;//�X�{

	//20151224�[
	bool GoodorBad;

	//20160616�ͥ[�A�e�S�x�I��
	bool theMAP;

};


class RANSAC
{
public:
	RANSAC();
	virtual ~RANSAC();

	double SetDeviation = 0.25;

	void Initial_RANSAC(double u0 , double v0 , double fu , double fv);
	void RandomSampleConsensus(vector<Keep_Feature> &map_feature, vector<Keep_Feature> last_map_feature, vector<Ransac_Pos>& ransac_match, int PhotoCount);
	void SelectWorldCoordinates(Keep_Feature &TheFeature);
	void UpdateWorldCoordinates(Ransac_Pos ransac_match , vector<Keep_Feature> &map_feature);

private:
	double l_u0;
	double l_v0;
	double l_fu;
	double l_fv;
};