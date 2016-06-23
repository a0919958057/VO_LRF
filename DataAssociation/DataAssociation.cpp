#include "DataAssociation.h"





DataAssociation::DataAssociation()
{
}

DataAssociation::~DataAssociation()
{
}

void DataAssociation::Match_Old_Feature_Original( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int search_window_size, const double original_d_match_threshold, const int extended )
{


	if( (int)map_feature.size()  &&  (int)feature.size() )
	{
		int elements = 0;
		if     ( extended == 2 )	elements = 16;
		else if( extended == 1 )	elements = 128;
		else if( extended == 0 )	elements = 64;


		double original_d = 0, original_dist1 = 0;
		int map_dist1_index = 0, feature_dist1_index = 0;


		for( int i=0 ; i < (int)map_feature.size() ; i++ )
		{

			if(map_feature[i].on_image)   continue;

			
			original_dist1 = 1e6;
			for( int j=0 ; j < (int)feature.size() ; j++ )
			{
				// 地圖地標在攝影機前方 and 判斷舊的地圖特徵點是否在預估區域內 and 滿足laplacian比對
				if( map_feature[i].at_front  &&
					map_feature[i].is_near  &&
					fabs( map_feature[i].ix_pre - feature[j].ix ) <= ( search_window_size - 1 )/2  &&  
					fabs( map_feature[i].iy_pre - feature[j].iy ) <= ( search_window_size - 1 )/2  &&  
					map_feature[i].laplacian == feature[j].laplacian )  
				{
					original_d = 0;
					for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
					{
						original_d += ( map_feature[i].original_descriptor[k] - feature[j].descriptor[k] )*( map_feature[i].original_descriptor[k] - feature[j].descriptor[k] );
					}
					original_d = sqrt(original_d);

					if( original_d < original_dist1 ) // 找出最小
					{
						original_dist1 = original_d;
						feature_dist1_index = j;
					}		
				}
			}
			if(original_dist1 > 1e5)   continue;

				
			original_dist1 = 1e6;	
			for( int m=0 ; m < (int)map_feature.size() ; m++ )
			{
				// 地圖地標在攝影機前方 and 判斷舊的地圖特徵點是否在預估區域內 and 滿足laplacian比對
				if( map_feature[m].at_front  &&
					map_feature[m].is_near  &&
				    fabs( map_feature[m].ix_pre - feature[feature_dist1_index].ix ) <= ( search_window_size - 1 )/2  &&  
					fabs( map_feature[m].iy_pre - feature[feature_dist1_index].iy ) <= ( search_window_size - 1 )/2  &&   
					map_feature[m].laplacian == feature[feature_dist1_index].laplacian )  
				{
					original_d = 0;
					for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
					{
						original_d += ( map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] )*( map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] );
					}
					original_d = sqrt(original_d);

					if( original_d < original_dist1 ) // 找出最小
					{
						original_dist1 = original_d;
						map_dist1_index = m;
					}		
				}
			}
			if(original_dist1 > 1e5)   continue;

				
			if( i == map_dist1_index  &&  original_dist1 < original_d_match_threshold ) 
			{
				map_feature[i].on_image = true;
				on_image_num++;

				map_feature[i].old_match = true;
				find_map_feature.push_back(i);

				feature[feature_dist1_index].on_map = true;			

				map_feature[i].ix		= feature[feature_dist1_index].ix;
				map_feature[i].iy		= feature[feature_dist1_index].iy;				
				map_feature[i].size		= feature[feature_dist1_index].size;
				map_feature[i].dir		= feature[feature_dist1_index].dir;
				map_feature[i].hessian	= feature[feature_dist1_index].hessian;
				memcpy( map_feature[i].current_descriptor, feature[feature_dist1_index].descriptor, sizeof(map_feature[i].current_descriptor) );

				map_feature[i].search_window_size = search_window_size;
			}


		} // for( int i=0 ; i < (int)map_feature.size() ; i++ )


	} // if( (int)map_feature.size()  &&  (int)feature.size() )


}



void DataAssociation::Match_Old_Feature_Current( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int search_window_size, const double current_d_match_threshold, const int extended )
{



	if( (int)map_feature.size()  &&  (int)feature.size() )
	{

		int elements = 0;
		if     ( extended == 2 )	elements = 16;
		else if( extended == 1 )	elements = 128;
		else if( extended == 0 )	elements = 64;


		double original_d = 0, original_dist1 = 0, current_d = 0, current_dist1 = 0;
		int original_dist1_index = 0, current_dist1_index = 0;
		int map_dist1_index = 0, feature_dist1_index = 0;


		for( int i=0 ; i < (int)map_feature.size() ; i++ )
		{

			if(map_feature[i].on_image)   continue;


			original_dist1 = current_dist1 = 1e6;
			for( int j=0 ; j < (int)feature.size() ; j++ )
			{
				// 地圖地標在攝影機前方 and 判斷舊的地圖特徵點是否在預估區域內 and 滿足laplacian比對
				if( map_feature[i].at_front  &&
					map_feature[i].is_near  &&
					fabs( map_feature[i].ix_pre - feature[j].ix ) <= ( search_window_size - 1 )/2  &&  
					fabs( map_feature[i].iy_pre - feature[j].iy ) <= ( search_window_size - 1 )/2  &&  
					map_feature[i].laplacian == feature[j].laplacian )  
				{
					original_d = current_d = 0;
					for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
					{
						original_d += ( map_feature[i].original_descriptor[k] - feature[j].descriptor[k] )*( map_feature[i].original_descriptor[k] - feature[j].descriptor[k] );
						current_d += ( map_feature[i].current_descriptor[k] - feature[j].descriptor[k] )*( map_feature[i].current_descriptor[k] - feature[j].descriptor[k] );
					}
					original_d = sqrt(original_d);
					current_d = sqrt(current_d);
	
					if( original_d < original_dist1 ) // 找出最小
					{
						original_dist1 = original_d;
						original_dist1_index = j;
					}		

					if( current_d < current_dist1 ) // 找出最小
					{
						current_dist1 = current_d;
						current_dist1_index = j;
					}
				}
			}
			if( original_dist1 > 1e5  ||  current_dist1 > 1e5 )   continue;


			if( original_dist1_index == current_dist1_index )
			{
				feature_dist1_index = original_dist1_index;
			}
			else
			{
				continue;
			}
				

			original_dist1 = current_dist1 = 1e6;	
			for( int m=0 ; m < (int)map_feature.size() ; m++ )
			{
				// 地圖地標在攝影機前方 and 判斷舊的地圖特徵點是否在預估區域內 and 滿足laplacian比對
				if( map_feature[m].at_front  &&
					map_feature[m].is_near  &&
				    fabs( map_feature[m].ix_pre - feature[feature_dist1_index].ix ) <= ( search_window_size - 1 )/2  &&  
					fabs( map_feature[m].iy_pre - feature[feature_dist1_index].iy ) <= ( search_window_size - 1 )/2  &&   
					map_feature[m].laplacian == feature[feature_dist1_index].laplacian )  
				{
					original_d = current_d = 0;
					for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
					{
						original_d += ( map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] )*(map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] );
						current_d += ( map_feature[m].current_descriptor[k] - feature[feature_dist1_index].descriptor[k] )*(map_feature[m].current_descriptor[k] - feature[feature_dist1_index].descriptor[k] );
					}
					original_d = sqrt(original_d);
					current_d = sqrt(current_d);
	
					if( original_d < original_dist1 ) // 找出最小
					{
						original_dist1 = original_d;
						original_dist1_index = m;
					}		
	
					if( current_d < current_dist1 ) // 找出最小
					{
						current_dist1 = current_d;
						current_dist1_index = m;
					}
				}
			}
			if( original_dist1 > 1e5  ||  current_dist1 > 1e5 )   continue;


			if( original_dist1_index == current_dist1_index )
			{
				map_dist1_index = original_dist1_index;
			}
			else
			{
				continue;
			}

				
			if( i == map_dist1_index  &&  ( original_dist1 < current_d_match_threshold  ||  current_dist1 < current_d_match_threshold ) ) 
			{
				map_feature[i].on_image = true;
				on_image_num++;

				map_feature[i].old_match = true;
				find_map_feature.push_back(i);

				feature[feature_dist1_index].on_map = true;			

				map_feature[i].ix		= feature[feature_dist1_index].ix;
				map_feature[i].iy		= feature[feature_dist1_index].iy;				
				map_feature[i].size		= feature[feature_dist1_index].size;
				map_feature[i].dir		= feature[feature_dist1_index].dir;
				map_feature[i].hessian	= feature[feature_dist1_index].hessian;
				memcpy( map_feature[i].current_descriptor, feature[feature_dist1_index].descriptor, sizeof(map_feature[i].current_descriptor) );

				map_feature[i].search_window_size = search_window_size;
			}


		} // for( int i=0 ; i < (int)map_feature.size() ; i++ )


	} // if( (int)map_feature.size()  &&  (int)feature.size() )
		

}



/* *********** 舊寫法 此函式已經用不到了 **************
void DataAssociation::Match_Old_Feature_Sequence( vector<SingleFeature>& feature, vector<MapFeature>& map_feature, vector<int>& find_map_feature, int& on_image_num, const int max_on_image_num, const double absolute_d_match_threshold, const int extended, bool& found )
{


	if( (int)map_feature.size()  &&  (int)feature.size() )
	{
	
		int elements = 0;
		if     ( extended == 2 )	elements = 16;
		else if( extended == 1 )	elements = 128;
		else if( extended == 0 )	elements = 64;


		double original_d = 0, original_dist1 = 0;
		int original_dist1_index = 0;
		int map_dist1_index = 0, feature_dist1_index = 0;


		vector<int> temp_find_map_feature;
		temp_find_map_feature.clear();

		vector<int> temp_feature_index;
		temp_feature_index.clear();


		if( (int)map_feature.size() >= max_on_image_num )
		{
			for( int s=0 ; s < ( on_image_num ? 1 : (int)map_feature.size() ) ; s++ )
			{


				for( int r=0 ; r < max_on_image_num ; r++ )
				{

					int i = s + r;

					if( i >= (int)map_feature.size() )   break;

					if(map_feature[i].on_image)   continue; // 地圖特徵在影像上


					original_dist1 = 1e6;
					for( int j=0 ; j < (int)feature.size() ; j++ )
					{
						// 滿足laplacian比對
						if( map_feature[i].at_front  &&
							map_feature[i].is_near  &&
						    map_feature[i].laplacian == feature[j].laplacian )  
						{
							original_d = 0;
							for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
							{
								original_d += ( map_feature[i].original_descriptor[k] - feature[j].descriptor[k] )*(map_feature[i].original_descriptor[k] - feature[j].descriptor[k] );
							}
							original_d = sqrt(original_d);

							if( original_d < original_dist1 ) // 找出最小
							{
								original_dist1 = original_d;
								feature_dist1_index = j;
							}		
						}
					}
					if(original_dist1 > 1e5)   continue;


					original_dist1 = 1e6;	
					for( int m=0 ; m < (int)map_feature.size() ; m++ )
					{
						// 滿足laplacian比對
						if( map_feature[m].at_front  &&
							map_feature[m].is_near  &&
							map_feature[m].laplacian == feature[feature_dist1_index].laplacian )  
						{
							original_d = 0;
							for( int k=0 ; k < elements ; k++ ) // 計算尤基里德距離
							{
								original_d += ( map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] )*(map_feature[m].original_descriptor[k] - feature[feature_dist1_index].descriptor[k] );
							}
							original_d = sqrt(original_d);

							if( original_d < original_dist1 ) // 找出最小
							{
								original_dist1 = original_d;
								map_dist1_index = m;
							}		
						}
					}
					if(original_dist1 > 1e5)   continue;

				
					if( i == map_dist1_index  &&  original_dist1 < absolute_d_match_threshold ) 
					{
						temp_find_map_feature.push_back(i);
						temp_feature_index.push_back(feature_dist1_index);
					}


				} // for( int r=0 ; r < max_on_image_num ; r++ ) 



				if( (int)temp_find_map_feature.size() >= 4 )
				{

					found = true;

		//			on_image_num = 0;

		//			find_map_feature.clear();

		//			for( int i=0 ; i < (int)map_feature.size() ; i++ )
		//			{
		//				map_feature[i].old_match = false;
		//				map_feature[i].on_image = false;
		//			}

					//for( int i=0 ; i < (int)feature.size() ; i++ )
					//{
					//	feature[i].on_map = false;
					//}

					for( int i=0 ; i < (int)temp_find_map_feature.size() ; i++ )
					{
						map_feature[temp_find_map_feature[i]].on_image = true;
						on_image_num++;
	
						map_feature[temp_find_map_feature[i]].old_match = true;
						find_map_feature.push_back(temp_find_map_feature[i]);
		
						feature[temp_feature_index[i]].on_map = true;
	
						map_feature[temp_find_map_feature[i]].ix		= feature[temp_feature_index[i]].ix;
						map_feature[temp_find_map_feature[i]].iy		= feature[temp_feature_index[i]].iy;				
						map_feature[temp_find_map_feature[i]].size		= feature[temp_feature_index[i]].size;
						map_feature[temp_find_map_feature[i]].dir		= feature[temp_feature_index[i]].dir;
						map_feature[temp_find_map_feature[i]].hessian	= feature[temp_feature_index[i]].hessian;		
					}

					break;	

				}
				else
				{
					temp_find_map_feature.clear();
					temp_feature_index.clear();			
				}
		

	
			} // for( int s=0 ; s < ( on_image_num ? 1 : (int)map_feature.size() ) ; s++ )
		} // if( map_feature.size() >= max_on_image_num )


	
	


		temp_find_map_feature.swap(vector<int>());
		temp_feature_index.swap(vector<int>());


	} // if( (int)map_feature.size()  &&  (int)feature.size() )

}

*/




void DataAssociation::Search_Pair_Feature( const vector<SingleFeature> l_feature, const vector<SingleFeature> r_feature, vector<PairFeature>& pair_feature, const double bino_d_match_threshold, const int extended )
{


	int elements = 0; 
	if     ( extended == 2 )	elements = 16;
	else if( extended == 1 )	elements = 128;
	else if( extended == 0 )	elements = 64;


	PairFeature temp;


	double d = 0, dist1 = 0;
	int l_dist1_index = 0, r_dist1_index = 0;
	
	for( int i=0 ; i < (int)l_feature.size() ; i++ )
	{
		
		dist1 = 1e6;
		for( int j=0 ; j < (int)r_feature.size() ; j++ )
		{
			//  滿足左右ix區域限制 and 滿足左右iy區域限制 and 滿足左右半徑差值 and 左右laplacian值相同
			if( l_feature[i].ix > r_feature[j].ix  &&  
				fabs( l_feature[i].iy - r_feature[j].iy ) <= 3  &&
				abs( cvRound(l_feature[i].size*1.2/9.*2) - cvRound(r_feature[j].size*1.2/9.*2) ) <= 2  &&
				l_feature[i].laplacian == r_feature[j].laplacian )
			{
				d = 0;
				for( int k=0 ; k < elements ; k++ )
				{
					d += ( l_feature[i].descriptor[k] - r_feature[j].descriptor[k] )*( l_feature[i].descriptor[k] - r_feature[j].descriptor[k] );
				}
				d = sqrt(d);

				if( d < dist1 )
				{
					dist1 = d;
					r_dist1_index = j;
				}
			}
		}
		if( dist1 > 1e5 )   continue;



		dist1 = 1e6;
		for( int m=0 ; m < (int)l_feature.size() ; m++ )
		{
			//  滿足左右ix區域限制 and 滿足左右iy區域限制 and 滿足左右半徑差值 and 左右laplacian值相同
			if( l_feature[m].ix > r_feature[r_dist1_index].ix  &&
				fabs( l_feature[m].iy - r_feature[r_dist1_index].iy ) <= 3  &&
				abs( cvRound(l_feature[m].size*1.2/9.*2) - cvRound(r_feature[r_dist1_index].size*1.2/9.*2) ) <= 2  &&
				l_feature[m].laplacian == r_feature[r_dist1_index].laplacian )
			{
				d = 0;
				for( int k=0 ; k < elements ; k++ )
				{
					d += ( l_feature[m].descriptor[k] - r_feature[r_dist1_index].descriptor[k] )*( l_feature[m].descriptor[k] - r_feature[r_dist1_index].descriptor[k] );
				}
				d = sqrt(d);

				if( d < dist1 )
				{
					dist1 = d;
					l_dist1_index = m;
				}
			}			
		}
		if( dist1 > 1e5 )   continue;



		if( i == l_dist1_index  &&  dist1 < bino_d_match_threshold ) // (左右) 滿足雙眼D_match門檻
		{
			memset( &temp, 0, sizeof(PairFeature) ); // temp 初始化為零
		
			temp.ix	       = l_feature[i].ix;
			temp.iy        = l_feature[i].iy;			
			temp.laplacian = l_feature[i].laplacian;
			temp.size	   = l_feature[i].size;
			temp.dir	   = l_feature[i].dir;
			temp.hessian   = l_feature[i].hessian;
			memcpy( temp.descriptor, l_feature[i].descriptor, sizeof(temp.descriptor) ); // 複製向量

			temp.r_ix      = r_feature[r_dist1_index].ix;
			temp.r_iy      = r_feature[r_dist1_index].iy;			

			temp.on_map	   = l_feature[i].on_map;
			temp.match     = false;

			pair_feature.push_back(temp);
		}


	} // for( int i=0 ; i < (int)l_feature.size() ; i++ )




}


