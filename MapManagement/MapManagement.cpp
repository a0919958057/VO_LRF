#include "MapManagement.h"





MapManagement::MapManagement()
{

}

MapManagement::~MapManagement()
{
	
}




void MapManagement::Initial_MapManagement(void)
{


	num = 0;



	char str[50];
	double value;



	fstream in_Camera( "Camera.txt", ios::in );
	if(!in_Camera)	exit(1);
	while( in_Camera >> str >> value )
	{
		if      ( !strcmp( str, "l_u0" ) )	 l_u0 = value;
		else if	( !strcmp( str, "l_fu" ) )   l_fu = value;
		else if ( !strcmp( str, "r_u0" ) )	 r_u0 = value;
		else if ( !strcmp( str, "r_fu" ) )   r_fu = value;
		else if ( !strcmp( str, "L" ) )      L    = value;
	}
	in_Camera.close();



	fstream in_Parameter( "Parameter.txt", ios::in );
	if(!in_Parameter)	exit(1);
	while( in_Parameter >> str >> value )
	{
		if	    ( !strcmp( str, "minimum_distance" ) )	minimum_distance = value;
		else if ( !strcmp( str, "maximum_distance" ) )	maximum_distance = value;

	}
	in_Parameter.close();
}









void MapManagement::GetFeatureData( const IplImage* image_gray, const CvSURFParams params, vector<SingleFeature>& feature )
{


	int elements = 0; 
	if     ( params.extended == 2 )	elements = 16;
	else if( params.extended == 1 )	elements = 128;
	else if( params.extended == 0 )	elements = 64;


	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq* imageKeypoints = NULL;
	CvSeq* imageDescriptors = NULL;
	

	if(params.extended == 2)
	{
		surf01.cvExtractSURF_cv200_16D_avg( image_gray, 0, &imageKeypoints, &imageDescriptors, storage, params, 0 );
	}
	else if( params.extended == 0  ||  params.extended == 1 )
	{
		surf01.cvExtractSURF_cv200_avg( image_gray, 0, &imageKeypoints, &imageDescriptors, storage, params, 0 );
	}

	CvSeqReader reader, kreader;
    cvStartReadSeq( imageKeypoints, &kreader );
    cvStartReadSeq( imageDescriptors, &reader );

	SingleFeature temp;

	for( int i=0 ; i < imageDescriptors->total ; i++ )
	{
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );

		memset( &temp, 0, sizeof(SingleFeature) );

		temp.ix	       = kp->pt.x;
		temp.iy	       = kp->pt.y;
		temp.laplacian = kp->laplacian;
		temp.size	   = kp->size;
		temp.dir	   = kp->dir;
		temp.hessian   = kp->hessian;

		for( int j=0 ; j < elements ; j++ )
		{
			temp.descriptor[j] = descriptor[j];
		}

		feature.push_back(temp);
	}

	cvReleaseMemStorage(&storage);
	
}







void MapManagement::Image_Correction( double& ix, double &iy, const double fu, const double fv, const double u0, const double v0, const double coefficient[8] )
{
	double hx_hz = (ix-u0)/fu;
	double hy_hz = (iy-v0)/fv;

    double r_dist = sqrt(hx_hz*hx_hz+hy_hz*hy_hz);

	double G = 4*coefficient[4]*r_dist*r_dist + 6*coefficient[5]*r_dist*r_dist*r_dist*r_dist + 8*coefficient[6]*hy_hz + 8*coefficient[7]*hx_hz + 1;

	double new_hx_hz =  hx_hz + (hx_hz*(coefficient[0]*r_dist*r_dist + coefficient[1]*r_dist*r_dist*r_dist*r_dist) + 2*coefficient[2]*hx_hz*hy_hz + coefficient[3]*(r_dist*r_dist + 2*hx_hz*hx_hz) )/G;
    double new_hy_hz =  hy_hz + (hy_hz*(coefficient[0]*r_dist*r_dist + coefficient[1]*r_dist*r_dist*r_dist*r_dist) + coefficient[2]*(r_dist*r_dist + 2*hy_hz*hy_hz) + 2*coefficient[3]*hx_hz*hy_hz )/G;
        
	ix = u0 + fu*new_hx_hz;
    iy = v0 + fv*new_hy_hz;
}




















void MapManagement::Erase_Bad_Feature( vector<MapFeature>& map_feature, vector<int>& erase_map_feature , int& on_image_num, const int max_on_image_num, const int erase_times, const double erase_ratio )
{


	int match_stable_on_image_num = 0;


	for( int i=0 ; i < (int)map_feature.size() ; i++ )
	{

		map_feature[i].run_times++;

		if(map_feature[i].on_image)
		{
			map_feature[i].detect_times++;
		}

		if( !map_feature[i].match_stable  &&  map_feature[i].run_times <= erase_times  &&  map_feature[i].detect_times >= cvRound(erase_times*erase_ratio) )
		{
			map_feature[i].match_stable = true; // 特徵點比對率符合

			map_feature[i].num = num;
			num++;
		}

		if( map_feature[i].on_image  &&  map_feature[i].match_stable )
		{
			match_stable_on_image_num++;
		}

	}



	if( match_stable_on_image_num >= max_on_image_num )
	{
		for( int i=0 ; i < (int)map_feature.size() ; i++ )
		{
			if(!map_feature[i].match_stable)
			{
				erase_map_feature.push_back(i); // 要刪除的地圖特徵點編號
			}
		}
	}
	else
	{
		for( int i=0 ; i < (int)map_feature.size() ; i++ )
		{
			if( map_feature[i].run_times == erase_times  &&  !map_feature[i].match_stable ) //特徵點比對率低
			{
				erase_map_feature.push_back(i); // 要刪除的地圖特徵點編號
			}
		}
	}
	



	for( int i=(int)erase_map_feature.size()-1 ; i >= 0 ; i-- ) // 執行刪除
	{
		if(map_feature[erase_map_feature[i]].on_image) // 只能放這裡 
		{
			on_image_num--;
		}

		map_feature.erase( map_feature.begin() + erase_map_feature[i] );
	}


}










void MapManagement::Add_New_Feature( vector<SingleFeature> feature, vector<PairFeature>& pair_feature, vector<MapFeature>& map_feature, vector<int>& add_map_feature, int& on_image_num, const int max_on_image_num, const int add_window_size, const double similar_threshold, const int extended )
{


	int elements = 0; 
	if     ( extended == 2 )	elements = 16;
	else if( extended == 1 )	elements = 128;
	else if( extended == 0 )	elements = 64;
	

	double original_d = 0, current_d = 0;
	double hz = 0;

	MapFeature temp;

	int lift = 0;

	for( int i=0 ; i < (int)pair_feature.size() ; i++ ) // 執行新增特徵點
	{



		lift = 0;
		for( int j=0 ; j < (int)map_feature.size() ; j++ )
		{
			if( ( map_feature[j].on_image  ||  !map_feature[j].match_stable )  &&  map_feature[j].ix<160 ) lift++;
		}


		//hz = L*((l_fu+r_fu)/2.)/( pair_feature[i].ix - pair_feature[i].r_ix );
		hz = L*l_fu*r_fu/( r_fu*(pair_feature[i].ix-l_u0) - l_fu*(pair_feature[i].r_ix-r_u0) ); // 特徵點在攝影機z座標


		// 影像特徵點小於限制 and 新擷取特徵點沒在地圖上 and 滿足新增範圍
		if( on_image_num < max_on_image_num  &&  !pair_feature[i].on_map  &&  hz > minimum_distance  &&  hz < maximum_distance ) 
		{

			for( int j=0 ; j < (int)map_feature.size() ; j++ ) // 新擷取特徵與影像上地圖特徵是否太靠近  新擷取特徵與地圖特徵是否太相似
			{

				if( lift > 4  &&  pair_feature[i].ix < 160 )
				{
					pair_feature[i].is_close = true;
					break;
				}

				if( //on_image_num >= max_on_image_num/2  &&
					( map_feature[j].on_image  ||  !map_feature[j].match_stable )  &&
					fabs( pair_feature[i].ix - map_feature[j].ix ) <= (add_window_size - 1)/2  &&  
					fabs( pair_feature[i].iy - map_feature[j].iy ) <= (add_window_size - 1)/2 )
				{
					pair_feature[i].is_close = true;
					break;
				}


				// 地圖上
				if( map_feature[j].at_front  &&
					map_feature[j].is_near  && 
					map_feature[j].ix_pre > 0-320/2  &&  map_feature[j].ix_pre < 320+320/2  &&  
					map_feature[j].iy_pre > 0-240/2  &&  map_feature[j].iy_pre < 240+240/2 )
				{
					original_d = current_d = 0;
					for( int k=0 ; k < elements ; k++ )
					{
						original_d += ( pair_feature[i].descriptor[k] - map_feature[j].original_descriptor[k] )*( pair_feature[i].descriptor[k] - map_feature[j].original_descriptor[k] );
						current_d += ( pair_feature[i].descriptor[k] - map_feature[j].current_descriptor[k] )*( pair_feature[i].descriptor[k] - map_feature[j].current_descriptor[k] );				
					}
					original_d = sqrt(original_d);
					current_d = sqrt(current_d);

					if( original_d <= similar_threshold  ||  current_d <= similar_threshold ) // 小於相似門檻
					{
						pair_feature[i].is_similar = true;
						break;
					}
				}
				

			} //for( int j=0 ; j < (int)map_feature.size() ; j++ )
				
		

			// 影像上
			for( int j=0 ; j < (int)feature.size() ; j++ )
			{	
				original_d = 0;
				for( int k=0 ; k < elements ; k++ )
				{
					original_d += ( pair_feature[i].descriptor[k] - feature[j].descriptor[k] )*( pair_feature[i].descriptor[k] - feature[j].descriptor[k] );			
				}
				if(!original_d)  continue;
				original_d = sqrt(original_d);


				if( original_d <= 0.15 ) // 小於相似門檻
				{
					pair_feature[i].is_similar = true;
					break;
				}
			}





			if( !pair_feature[i].is_close  &&  !pair_feature[i].is_similar )
			{
				memset( &temp, 0, sizeof(MapFeature) ); // temp 初始化為零
					
				temp.ix	       = pair_feature[i].ix;
				temp.iy        = pair_feature[i].iy;			
				temp.laplacian = pair_feature[i].laplacian;
				temp.size	   = pair_feature[i].size;
				temp.dir	   = pair_feature[i].dir;
				temp.hessian   = pair_feature[i].hessian;
				memcpy( temp.original_descriptor, pair_feature[i].descriptor, sizeof(temp.original_descriptor) ); // 複製向量
	
				temp.r_ix      = pair_feature[i].r_ix;
				temp.r_iy      = pair_feature[i].r_iy;

				//temp.num = num;
				//num++;

				temp.on_image = true;
				on_image_num++;

				temp.new_add = true;
				add_map_feature.push_back((int)map_feature.size()); // 要新增的地圖特徵點編號

				map_feature.push_back(temp);
			}
				

		}


	} // for( int i=0 ; i < (int)pair_feature.size() ; i++ ) 


}





























