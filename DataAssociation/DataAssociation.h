
#include <cv.h>

#include <vector>
#include <fstream>
#include "..//MapManagement//MapManagement.h"
using namespace std;

#ifndef _DATAASSOCIATION_h_
#define _DATAASSOCIATION_h_



class DataAssociation
{

private:


public:

	DataAssociation();
	virtual ~DataAssociation();

	void Match_Old_Feature_Original( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int search_window_size, const double original_d_match_threshold, const int extended );
	void Match_Old_Feature_Current( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int search_window_size, const double current_d_match_threshold, const int extended );
//	void Match_Old_Feature_Sequence( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int max_on_image_num, const double absolute_d_match_threshold, const int extended, bool& found );
	void Search_Pair_Feature( const vector<SingleFeature> l_feature, const vector<SingleFeature> r_feature, vector<PairFeature>& pair_feature, const double bino_d_match_threshold, const int extended );


};


#endif