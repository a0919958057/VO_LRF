#include "SLAM.h"
#include "time.h"

#include <ctime>



SLAM::SLAM()
{

}

SLAM::~SLAM()
{

}




// SLAM 初始化
void SLAM::Initial_SLAM(void)
{
	R_matrix = (CvMat*)cvLoad("R_matrix.xml");
	T_matrix = (CvMat*)cvLoad("T_matrix.xml");

	found = false;
	lost = false;
	count = 0;
	hessian_threshold_subtract = 0;
	hessian_threshold_add = 0;

	char str[50];
	double value;

	fstream in_Parameter( "Parameter.txt", ios::in );
	if(!in_Parameter)	exit(1);
	while( in_Parameter >> str >> value )
	{
		if	    ( !strcmp( str, "extended" ) )					  extended			    	 = (int)value;
		else if ( !strcmp( str, "hessian_threshold" ) )			  hessian_threshold		     = (int)value;
		else if ( !strcmp( str, "octaves" ) )					  octaves					 = (int)value;
		else if ( !strcmp( str, "octave_layers" ) )				  octave_layers			     = (int)value;
		else if	( !strcmp( str, "hessian_error" ) )				  hessian_error			     = (int)value;
		else if ( !strcmp( str, "minimum_hessian_threshold" ) )	  minimum_hessian_threshold	 = (int)value;
		else if ( !strcmp( str, "maximum_hessian_threshold" ) )	  maximum_hessian_threshold	 = (int)value;
		else if ( !strcmp( str, "minimum_Search_Window_Size" ) )  minimum_Search_Window_Size = (int)value;
		else if ( !strcmp( str, "maximum_Search_Window_Size" ) )  maximum_Search_Window_Size = (int)value;
		else if ( !strcmp( str, "add_window_size" ) )			  add_window_size			 = (int)value;
		else if ( !strcmp( str, "bino_d_match_threshold" ) )	  bino_d_match_threshold	 = value;
		else if ( !strcmp( str, "original_d_match_threshold" ) )  original_d_match_threshold = value;
		else if ( !strcmp( str, "current_d_match_threshold" ) )	  current_d_match_threshold  = value;
		else if ( !strcmp( str, "absolute_d_match_threshold" ) )  absolute_d_match_threshold = value;
		else if ( !strcmp( str, "similar_threshold" ) )			  similar_threshold	         = value;
		else if ( !strcmp( str, "erase_times" ) )			      erase_times			     = (int)value;
		else if ( !strcmp( str, "erase_ratio" ) )			      erase_ratio			     = value;
		else if ( !strcmp( str, "max_on_image_num" ) )	          max_on_image_num			 = (int)value;
		else if ( !strcmp( str, "Map_Covariance_Predict" ) )	  Map_Covariance_Predict	 = (int)value;
		else if ( !strcmp( str, "maximum_distance" ) )            maximum_distance           = value;
		else if ( !strcmp( str, "Initial_State_Covariance" ) )	  Initial_State_Covariance	 = value;
		else if ( !strcmp( str, "Sigma_wv" ) )			          Sigma_wv					 = value;
		else if ( !strcmp( str, "Sigma_ww" ) )				      Sigma_ww					 = value;
		else if ( !strcmp( str, "Sigma_v" ) )				      Sigma_v					 = value;
	}
	in_Parameter.close();




	fstream in_Camera( "Camera.txt", ios::in );
	if(!in_Camera)	exit(1);
	while( in_Camera >> str >> value )
	{ 
		if ( !strcmp( str, "l_u0" ) )	                      l_u0						= value;
		else if ( !strcmp( str, "l_v0" ) )						  l_v0						= value;
		else if ( !strcmp( str, "l_fu" ) )                        l_fu						= value;
		else if ( !strcmp( str, "l_fv" ) )                        l_fv						= value;
		else if ( !strcmp( str, "l_coefficient[0]" ) )			  l_coefficient[0]			= value;
		else if ( !strcmp( str, "l_coefficient[1]" ) )			  l_coefficient[1]			= value;
		else if ( !strcmp( str, "l_coefficient[2]" ) )			  l_coefficient[2]			= value;
		else if ( !strcmp( str, "l_coefficient[3]" ) )			  l_coefficient[3]			= value;
		else if ( !strcmp( str, "l_coefficient[4]" ) )			  l_coefficient[4]			= value;
		else if ( !strcmp( str, "l_coefficient[5]" ) )			  l_coefficient[5]			= value;
		else if ( !strcmp( str, "l_coefficient[6]" ) )			  l_coefficient[6]			= value;
		else if ( !strcmp( str, "l_coefficient[7]" ) )			  l_coefficient[7]			= value;
		else if ( !strcmp( str, "r_u0" ) )						  r_u0						= value;
		else if ( !strcmp( str, "r_v0" ) )						  r_v0						= value;
		else if ( !strcmp( str, "r_fu" ) )                        r_fu						= value;
		else if ( !strcmp( str, "r_fv" ) )                        r_fv						= value;
		else if ( !strcmp( str, "r_coefficient[0]" ) )			  r_coefficient[0]			= value;
		else if ( !strcmp( str, "r_coefficient[1]" ) )			  r_coefficient[1]			= value;
		else if ( !strcmp( str, "r_coefficient[2]" ) )			  r_coefficient[2]			= value;
		else if ( !strcmp( str, "r_coefficient[3]" ) )			  r_coefficient[3]			= value;
		else if ( !strcmp( str, "r_coefficient[4]" ) )			  r_coefficient[4]			= value;
		else if ( !strcmp( str, "r_coefficient[5]" ) )			  r_coefficient[5]			= value;
		else if ( !strcmp( str, "r_coefficient[6]" ) )			  r_coefficient[6]			= value;
		else if ( !strcmp( str, "r_coefficient[7]" ) )			  r_coefficient[7]			= value;
		else if ( !strcmp( str, "L" ) )                           L					    	= value;
	}
	in_Camera.close();

	PhotoCount=0;
	feature_num=0;
	p3dx_x.x=0;
	p3dx_x.x_dir=0;
	p3dx_x.y=0;
	p3dx_x.y_dir=0;
	p3dx_x.z=0;
	p3dx_x.z_dir=0;
	ransac_time=0;

	map.Initial_MapManagement();

}


bool Hessian_Strength( const SingleFeature& temp1, const SingleFeature& temp2 ) // 比大小函式(Search_Feature用)
{
	return temp1.hessian > temp2.hessian;
}


// 執行 BinocularSLAM
void SLAM::Run_BinocularSLAM( const IplImage* l_image, const IplImage* r_image, const double sample_time ,bool isrunning,bool isturnning)
{
	  	//fstream app_point("point.txt",ios::app);
		//app_point << "ok1" << endl;

		IplImage* l_image_gray = cvCreateImage( cvGetSize(l_image), IPL_DEPTH_8U, 1 );
		cvCvtColor( l_image, l_image_gray, CV_BGR2GRAY ); 

		IplImage* r_image_gray = cvCreateImage( cvGetSize(r_image), IPL_DEPTH_8U, 1 );
		cvCvtColor( r_image, r_image_gray, CV_BGR2GRAY ); 


		vector<SingleFeature> l_feature;
		l_feature.clear();
		vector<SingleFeature> r_feature;
		r_feature.clear();

		vector<PairFeature> pair_feature;
		pair_feature.clear();

		//app_point << "ok2" << endl;

		Find_Feature( l_image_gray , r_image_gray , l_feature , r_feature );
		//app_point << "ok3" << endl;

		Comparison_Feature( l_feature , r_feature , pair_feature, isrunning,isturnning);
		//app_point << "ok4" << endl;

		//Ransac_Feature();

		fstream app_state("txt\\feature_state.txt",ios::app);
		app_state<<"l_feature SIZE  "<<l_feature.size()<<"  r_feature SIZE  "<<r_feature.size()<<endl;
		app_state.close();
		//
		Save_Feature( pair_feature );
		//app_point << "ok5" << endl;

		cvReleaseImage(&r_image_gray);
		cvReleaseImage(&l_image_gray);
		l_feature.swap(vector<SingleFeature>());
		r_feature.swap(vector<SingleFeature>());
		pair_feature.swap(vector<PairFeature>());

		//app_point << "ok6" << endl<< endl;

		//SLAM_Set_Hessian_Threshold();

		//app_point.close();

		PhotoCount++;
	


}

void SLAM::Find_Feature(const IplImage* l_image_gray , const IplImage* r_image_gray , vector<SingleFeature>& l_feature , vector<SingleFeature>& r_feature )
{
	CvSURFParams params;
	params.extended = extended;
	params.hessianThreshold = hessian_threshold;
	params.nOctaves = octaves;
	params.nOctaveLayers = octave_layers;

	map.GetFeatureData( l_image_gray, params, l_feature );
	sort( l_feature.begin(), l_feature.end(), Hessian_Strength ); // hessian值由大到小排序
	for( int i=0 ; i < (int)l_feature.size() ; i++ )
	{
		map.Image_Correction( l_feature[i].ix, l_feature[i].iy, l_fu, l_fv, l_u0, l_v0, l_coefficient );
	}

	map.GetFeatureData( r_image_gray, params, r_feature );
	sort( r_feature.begin(), r_feature.end(), Hessian_Strength ); // hessian值由大到小排序
	for( int i=0 ; i < (int)r_feature.size() ; i++ )
	{
		map.Image_Correction( r_feature[i].ix, r_feature[i].iy, r_fu, r_fv, r_u0, r_v0, r_coefficient );
	}
}

int RANSACTIME = 10; //----------------------------------------------------------------------------------------------

void SLAM::Comparison_Feature( vector<SingleFeature>&l_feature , vector<SingleFeature>&r_feature , vector<PairFeature>&pair_feature , bool isMoveDone,bool isHeadingDone)
{
	//fstream app_point("point.txt",ios::app);
	fstream app_p3dx("isruning.txt",ios::app);
	fstream app_last_map_feature("last_map_feature.txt",ios::app);
	fstream app_p3dxpos("p3dxpos.txt",ios::app);
	
	//app_point << "ok3_0" << endl;

	for(int i=0 ; i<map_feature.size() ; i++)
	{
		map_feature[i].match = false;
	}

	if( l_feature.size() && r_feature.size() )
	{
		//app_point << "ok3_1" << endl;

		Keep_Feature temp;

		vector<int> add_map_feature;
		add_map_feature.clear();

		datamatch.Search_Pair_Feature( l_feature, r_feature, pair_feature, bino_d_match_threshold, extended );

		//app_point << "ok3_2" << endl;

		if (PhotoCount == RANSACTIME || (PhotoCount >= RANSACTIME & PhotoCount % RANSACTIME == 0))
		//if(!(isMoveDone&&isHeadingDone) && firstmove) //首走
		{
			last_map_feature=map_feature;
			firstmove=false;
			firststop=true;	

			stoptime=0;
		}

		Comparison_Feature_original_descriptor( pair_feature, isMoveDone,isHeadingDone );
		//app_point << "ok3_3" << endl;
        
		if(PhotoCount<2) firststop=false;

		if (PhotoCount == RANSACTIME || (PhotoCount >= RANSACTIME & PhotoCount % RANSACTIME == 0))
		//if((isMoveDone&&isHeadingDone) && firststop && stoptime==5) //首停
		{
			ransac_time++;
			Comparison_Feature_last_map_feature_descriptor( pair_feature );//比對last_map_feature 特徵

			///
			app_last_map_feature<<ransac_time<<"  "<<  PhotoCount<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<endl;
			for(int i=0 ; i<last_map_feature.size() ; i++)
			{
				if(!last_map_feature[i].match) continue;
				app_last_map_feature << setw(8) << last_map_feature[i].num << "  " << setw(8) << last_map_feature[i].hx << "  " << setw(8) << last_map_feature[i].hy  << "  " << setw(8) << last_map_feature[i].hz << "  " ;
				app_last_map_feature << setw(8) << last_map_feature[i].l_ix << "  " << setw(8) << last_map_feature[i].l_iy << "  " << setw(8) << last_map_feature[i].r_ix << "  " << setw(8) << last_map_feature[i].r_iy << endl;
			}
			app_last_map_feature << endl << endl;
			app_last_map_feature.close();

			///

			firststop=false;
			if(last_map_feature.size()>3) RANSAC();  //比對到特徵大於三個才做RANSAC

			if(ransac_match.size())
			{
				P3dx_Pos temppos;
	
				
				temppos.x=ransac_match[0].X[0];
				temppos.y=ransac_match[0].X[1];
				temppos.z=ransac_match[0].X[2];

				temppos.z_dir=asin(-ransac_match[0].Rs[0][2]);
				temppos.x_dir=atan2(ransac_match[0].Rs[1][2],ransac_match[0].Rs[2][2])-90*PI/180;
				temppos.y_dir=atan2(ransac_match[0].Rs[0][1],ransac_match[0].Rs[0][0]);
			
				
				p3dxpos.push_back(temppos);

				//1d
				/*
				p3dx_x.x=p3dx_x.x+temppos.y*sin(-p3dx_x.z_dir);
				p3dx_x.y=p3dx_x.y+temppos.y*cos(p3dx_x.z_dir);
				p3dx_x.z=p3dx_x.z+temppos.z;
				*/


				//2d
				p3dx_x.x = p3dx_x.x + temppos.y*sin(-temppos.z_dir) + temppos.x*cos(temppos.z_dir);
				p3dx_x.y = p3dx_x.y + temppos.y*cos(temppos.z_dir) + temppos.x*sin(temppos.z_dir);
				p3dx_x.z=p3dx_x.z+temppos.z;

				

				/*p3dx_x.z_dir=p3dx_x.z_dir+asin(-ransac_match[0].Rs[2][0]);
				p3dx_x.x_dir=p3dx_x.x_dir+atan2(ransac_match[0].Rs[2][1],ransac_match[0].Rs[2][2]);
				p3dx_x.y_dir=p3dx_x.y_dir+atan2(ransac_match[0].Rs[1][0],ransac_match[0].Rs[0][0]);*/

			//3d
			//	p3dx_x.x=p3dx_x.x+temppos.x*(cos(p3dx_x.z_dir)*cos(p3dx_x.y_dir))+temppos.y*(cos(p3dx_x.z_dir)*sin(p3dx_x.y_dir)*sin(p3dx_x.x_dir)-sin(p3dx_x.z_dir)*cos(p3dx_x.x_dir))+temppos.z*(cos(p3dx_x.z_dir)*sin(p3dx_x.y_dir)*cos(p3dx_x.x_dir)+sin(p3dx_x.z_dir)*sin(p3dx_x.x_dir));
			//	p3dx_x.y=p3dx_x.y+temppos.x*(sin(p3dx_x.z_dir)*cos(p3dx_x.y_dir))+temppos.y*(-sin(p3dx_x.z_dir)*sin(p3dx_x.y_dir)*sin(p3dx_x.x_dir)+cos(p3dx_x.z_dir)*cos(p3dx_x.x_dir))+temppos.z*(-sin(p3dx_x.z_dir)*sin(p3dx_x.y_dir)*cos(p3dx_x.x_dir)-cos(p3dx_x.z_dir)*sin(p3dx_x.x_dir));
			//	p3dx_x.z=p3dx_x.z+temppos.x*(-sin(p3dx_x.z_dir))+temppos.y*(cos(p3dx_x.y_dir)*sin(p3dx_x.x_dir))+temppos.z*(cos(p3dx_x.y_dir)*cos(p3dx_x.x_dir));
			
				
				p3dx_x.z_dir=p3dx_x.z_dir+temppos.z_dir;
				p3dx_x.x_dir=p3dx_x.x_dir+temppos.x_dir;
				p3dx_x.y_dir=p3dx_x.y_dir+temppos.y_dir;


				app_p3dxpos<<ransac_time<<"  "<<PhotoCount<<"  0"<<"  0"<<"  0"<<"  0"<<endl;
				app_p3dxpos<<p3dxpos[p3dxpos.size()-1].x<<"   "<<p3dxpos[p3dxpos.size()-1].y<<"   "<<p3dxpos[p3dxpos.size()-1].z<<"   ";
				app_p3dxpos << p3dxpos[p3dxpos.size() - 1].x_dir * 180 / PI << "   " << p3dxpos[p3dxpos.size() - 1].y_dir * 180 / PI << "   " << p3dxpos[p3dxpos.size() - 1].z_dir * 180 / PI << endl;
				app_p3dxpos<<p3dx_x.x<<"   "<<p3dx_x.y<<"   "<<p3dx_x.z<<"   ";
				app_p3dxpos << p3dx_x.x_dir * 180 / PI << "   " << p3dx_x.y_dir * 180 / PI << "   " << p3dx_x.z_dir * 180 / PI << endl << endl;


			}
			else 
			{
				P3dx_Pos temppos;

				temppos.x=0;
				temppos.y=0;
				temppos.z=0;

				temppos.z_dir=0;
				temppos.x_dir=0;
				temppos.y_dir=0;

				p3dxpos.push_back(temppos);

				app_p3dxpos<<ransac_time<<"  "<<PhotoCount<<"  0"<<"  0"<<"  0"<<"  0"<<endl;
				app_p3dxpos<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<endl;
				app_p3dxpos<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<endl;

			}


			ransac_match.clear();
			last_map_feature.clear();
			stoptime=0;
		    //last_map_feature = map_feature;
		}
		//app_point << "ok3_4" << endl;
////////////
	/*fstream app_descriptor("txt\\feature_descriptor.txt",ios::app);
    fstream app_state("txt\\feature_state.txt",ios::app);
	app_descriptor << setw(15) << PhotoCount << setw(15) << map_feature.size();
	app_state << setw(15) << PhotoCount << setw(15) << map_feature.size();
	for( int i=0 ; i<18 ; i++ )
	{
		app_descriptor << setw(15) << "0";
	}
	app_descriptor << endl;
	app_state << endl;

	for(int i=0 ; i<map_feature.size() ; i++)
	{
		app_descriptor <<setw(3) << map_feature[i].num << setw(14) << map_feature[i].l_ix << setw(15) << map_feature[i].l_iy ;
		app_state <<setw(3) << map_feature[i].num<< setw(8)<<map_feature[i].stable<< setw(8) << map_feature[i].count << setw(10) << map_feature[i].filter;
		for( int j=0 ; j<16 ; j++ )
		{
			app_descriptor << setw(15) << map_feature[i].current_descriptor[j];
		}

		app_descriptor << endl;
		app_state<< endl;
	}
	app_descriptor << endl;
	app_state<< endl;
	app_descriptor.close();
	app_state.close();

	//////////*/
		add_map_feature.swap(vector<int>());
	}
	//app_point << "ok3_5" << endl;
	//app_point.close();
	app_p3dx.close();
	app_p3dxpos.close();

}
void SLAM::Comparison_Feature_original_descriptor( vector<PairFeature> &pair_feature ,bool isMoveDone,bool isHeadingDone)
{
	//fstream app_point("point.txt",ios::app);
	fstream app_Comparison("Comparison.txt",ios::app);
	//app_Comparison <<"PhotoCount"<< PhotoCount <<"   original"<< endl;
	//app_point<< "  ok3_2_0" << endl;

	if( map_feature.size() && pair_feature.size() )
	{
		//app_point << "ok3_2_1" << endl;
		for(int i=0 ; i<map_feature.size() ; i++)
		{
			if(map_feature[i].match) continue;

			double dist=1e6;
			int match_num=0 , match_num2=0; 
			//app_point << "ok3_2_2" << endl;

			for(int j=0 ; j<pair_feature.size() ; j++)
			{				
				if( map_feature[i].laplacian == pair_feature[j].laplacian )
				{
					double dist1=0;
					for(int k=0 ; k<16 ; k++)
					{
						dist1 += (map_feature[i].original_descriptor[k] - pair_feature[j].descriptor[k])*(map_feature[i].original_descriptor[k] - pair_feature[j].descriptor[k]);
					}
					//app_point << "ok3_2_3" << endl;

					dist1=sqrt(dist1);

					if(dist1 < dist)
					{
						dist = dist1;
						match_num = j;
					}
				}
			}
			//app_point << "ok3_2_4    " << endl;
            //app_Comparison<< dist <<endl;
			if(dist > 1e5) continue;

			dist=1e6;
			for(int j=0 ; j<map_feature.size() ; j++)
			{
				if( map_feature[j].laplacian == pair_feature[match_num].laplacian )
				{
					double dist1=0;
					for(int k=0 ; k<16 ; k++)
					{
						dist1 += (map_feature[j].original_descriptor[k] - pair_feature[match_num].descriptor[k])*(map_feature[j].original_descriptor[k] - pair_feature[match_num].descriptor[k]);
					}
					//app_point << "ok3_2_5" << endl;

					dist1=sqrt(dist1);

					if(dist1 < dist)
					{
						dist = dist1;
						match_num2 = j;
					}
				}
			}
            //app_point<< "ok3_2_6    "<<endl;
			//app_Comparison  << dist << setw(5) << i << setw(5) << match_num2 << endl;


			if( i==match_num2 && dist<0.2 )
			{

				if (map_feature[i].times<=10) //十次
				{
					map_feature[i].times++;

                    if ((fabs(map_feature[i].l_ix-pair_feature[match_num].ix)<15) && (fabs(map_feature[i].l_iy-pair_feature[match_num].iy)<15))
					{
							map_feature[i].filter<<=1;
							map_feature[i].filter++;
							
					}
					else
						map_feature[i].filter<<=1;
				}
				if (map_feature[i].times>=10)
				{
					map_feature[i].filter=map_feature[i].filter %256;  //限定大小

						if ((fabs(map_feature[i].l_ix-pair_feature[match_num].ix)<15) && (fabs(map_feature[i].l_iy-pair_feature[match_num].iy)<15))
						{
							map_feature[i].filter<<=1;
							map_feature[i].filter++;
						}
						else
						{
						map_feature[i].filter<<=1;
						}
                        map_feature[i].stable=0;
						for(int j=0;j<8;j++) //計算八次內穩定幾次
						{
							if ((map_feature[i].filter>>j)&1==1)
								map_feature[i].stable++;
						}
						if (map_feature[i].stable<5)
							map_feature[i].count++;  //不穩定計數+1
				}

				
                	map_feature[i].l_ix = pair_feature[match_num].ix;
				    map_feature[i].l_iy = pair_feature[match_num].iy;
				    map_feature[i].r_ix = pair_feature[match_num].r_ix;
				    map_feature[i].r_iy = pair_feature[match_num].r_iy;
					map_feature[i].dir = pair_feature[match_num].dir;
					map_feature[i].size = pair_feature[match_num].size;
					map_feature[i].hessian = pair_feature[match_num].hessian;
					map_feature[i].laplacian = pair_feature[match_num].laplacian;
					map_feature[i].appear = PhotoCount;

					if (PhotoCount == RANSACTIME || (PhotoCount >= RANSACTIME & PhotoCount % RANSACTIME == 0))
					//if(isMoveDone&&isHeadingDone) //車子未動時更新三維座標
					{
						firstmove=true;

						double hz=0,hy=0;
					  	double gxL = (map_feature[i].l_ix - l_u0)/l_fu;
						double gyL = -(map_feature[i].l_iy - l_v0)/l_fv;
						double gxR = (map_feature[i].r_ix - r_u0)/r_fu;

						double k = ( cvmGet(R_matrix,0,0) - cvmGet(R_matrix,2,0)*gxR )*gxL +( cvmGet(R_matrix,0,1) - cvmGet(R_matrix,2,1)*gxR )*gyL + ( cvmGet(R_matrix,0,2) - cvmGet(R_matrix,2,2)*gxR ) ;
						map_feature[i].hz = ( cvmGet(T_matrix,0,2)*gxR - cvmGet(T_matrix,0,0))/(k*1000);
		    			map_feature[i].hx = map_feature[i].hz*gxL;
						map_feature[i].hy = map_feature[i].hz*gyL;

						
						hz =map_feature[i].hz;
						hy = map_feature[i].hy;
						map_feature[i].hz = -hy;
						map_feature[i].hy = hz;
					}

				//app_point << "ok3_2_7" << endl;

				map_feature[i].match = true;
				pair_feature[match_num].match = true;
				///描述向量更新
				for(int j=0 ; j<16 ; j++)
				{
					map_feature[i].original_descriptor[j] = (map_feature[i].original_descriptor[j]+pair_feature[match_num].descriptor[j])/2;
				}
				///
				for(int j=0 ; j<16 ; j++)
				{
					map_feature[i].current_descriptor[j] = pair_feature[match_num].descriptor[j];
				}
			}
		}
        //int in_map_feature=0;
        vector<Keep_Feature> temp1;
		temp1=map_feature;
		map_feature.clear();

	        for(int k=0 ; k<temp1.size() ; k++)
		    {
			    if (temp1[k].count<10 && (PhotoCount-temp1[k].appear)<15)//不穩定次數大於10 或 連續15張未出現  則踢掉特徵 
		        {
		           map_feature.push_back(temp1[k]);
		        }
		
		    }//*/
		
             
		//app_point << "ok3_2_8" << endl << endl;

	}

	if(isMoveDone&&isHeadingDone)stoptime++;
	//app_point.close();
	app_Comparison.close();
}
void SLAM::Comparison_Feature_last_map_feature_descriptor( vector<PairFeature> &pair_feature )
{
	//fstream app_point("point.txt",ios::app);
	fstream app_Comparison("Comparison.txt",ios::app);
	//app_point  << "  ok3_3_0" << endl;

	if( last_map_feature.size() && pair_feature.size() )
	{
		//app_point << "ok3_3_1" << endl;
		for(int i=0 ; i<last_map_feature.size() ; i++)
		{
			if(last_map_feature[i].match) continue;

			double dist=1e6;
			int match_num=0 , match_num2=0; 
			//app_point << "ok3_3_2" << endl;

			for(int j=0 ; j<pair_feature.size() ; j++)
			{				
				if( last_map_feature[i].laplacian == pair_feature[j].laplacian )
				{
					double dist1=0;
					for(int k=0 ; k<16 ; k++)
					{
						dist1 += (last_map_feature[i].current_descriptor[k] - pair_feature[j].descriptor[k])*(last_map_feature[i].current_descriptor[k] - pair_feature[j].descriptor[k]);
					}
					//app_point << "ok3_3_3" << endl;

					dist1=sqrt(dist1);

					if(dist1 < dist)
					{
						dist = dist1;
						match_num = j;
					}
				}
			}
			//app_point << "ok3_3_4" << endl;;
			if(dist > 1e5) continue;

			dist=1e6;
			for(int j=0 ; j<last_map_feature.size() ; j++)
			{
				if( last_map_feature[j].laplacian == pair_feature[match_num].laplacian )
				{
					double dist1=0;
					for(int k=0 ; k<16 ; k++)
					{
						dist1 += (last_map_feature[j].current_descriptor[k] - pair_feature[match_num].descriptor[k])*(last_map_feature[j].current_descriptor[k] - pair_feature[match_num].descriptor[k]);
					}
					//app_point << "ok3_3_5" << endl;

					dist1=sqrt(dist1);

					if(dist1 < dist)
					{
						dist = dist1;
						match_num2 = j;
					}
				}
			}

			//app_point << "ok3_3_6    "<<endl; 

			if( i==match_num2 && dist<0.2 )
			{
					last_map_feature[i].l_ix = pair_feature[match_num].ix;
				    last_map_feature[i].l_iy = pair_feature[match_num].iy;
				    last_map_feature[i].r_ix = pair_feature[match_num].r_ix;
				    last_map_feature[i].r_iy = pair_feature[match_num].r_iy;


				//app_point << "ok3_3_7" << endl;

				last_map_feature[i].match = true;
				//pair_feature[match_num].match = true;

				for(int j=0 ; j<16 ; j++)
				{
					last_map_feature[i].current_descriptor[j] = pair_feature[match_num].descriptor[j];
				}
			}
		}
		vector<Keep_Feature> temp2;
		temp2=last_map_feature;
		last_map_feature.clear();

	        for(int k=0 ; k<temp2.size() ; k++)
		    {
			    if (temp2[k].match )
		        {
		           last_map_feature.push_back(temp2[k]);
		        }
		
		    }//*/
		//app_point << "ok3_3_8" << endl;

	}
	//app_point << "ok3_3_9" << endl;
	//app_point.close();
	app_Comparison.close();
	
}



void SLAM::Save_Feature( vector<PairFeature>&pair_feature )
{
	//fstream app_point("point.txt",ios::app);
    fstream app_state("txt\\feature_state.txt",ios::app);
	//app_point << "ok4_0" << endl;

	on_image_num=0;

    for (int i=0;i<3;i++)
        location[i]=0;

	//搜尋影像上特徵分區
	for(int i=0 ; i<map_feature.size() ; i++)
	{
		if(map_feature[i].match)
		{
			on_image_num++;
		
		if(int(map_feature[i].l_ix/106)==0)
			location[0]++;
		else if(int(map_feature[i].l_ix/106)==1)
			location[1]++;
		else location[2]++;
		}
	}
	//
    //app_state<<"l_feature SIZE  "<<l_feature.size()<<"  r_feature SIZE  "<<r_feature.size()<<endl;
	app_state<<"pair_feature SIZE  "<<pair_feature.size()<<endl;
    for (int i=0;i<3;i++)
	{
		app_state<<"location["<<i<<"]" <<setw(2)<<location[i]<<endl;
	}
	app_state<<endl;
	//
	//app_point << "ok4_1" << endl;
   	for (int g=0;g<3;g++)
	{
			if( location[g]<10 )
			{
				Keep_Feature temp;

				//app_point << "ok4_2" << endl;

				for(int i=0 ; i<pair_feature.size() && location[g]<10 ; i++) //|| !(location1>0 &&location2 >0 && location3>0 )
				{
					bool is_near = false , is_similar = false,locat=false;
					for(int j=0 ; j<map_feature.size() ; j++)
					{
						////判斷特徵點是否太相近
						if( ((fabs( pair_feature[i].ix - map_feature[j].l_ix ) <= (add_window_size - 1)/3  &&  
							fabs( pair_feature[i].iy - map_feature[j].l_iy ) <= (add_window_size - 1)/3))  
							//|| fabs( pair_feature[i].r_ix - map_feature[j].l_ix ) <= (add_window_size - 1)/3   
							//||fabs( pair_feature[i].r_iy - map_feature[j].l_iy ) <= (add_window_size - 1)/3  
							)
						{
							is_near = true;
						}
						//app_point << "ok4_3" << endl;

						double dist = 1e6;

						for(int k=0 ; k<16 ; k++)
						{
							dist += (pair_feature[i].descriptor[k] - map_feature[j].original_descriptor[k])*(pair_feature[i].descriptor[k] - map_feature[j].original_descriptor[k]);
						}

						dist = sqrt(dist);

						if( dist < similar_threshold ) is_similar = true;
						//app_point << "ok4_4" << endl;

						dist = 1e6;

						for(int k=0 ; k<16 ; k++)
						{
							dist += (pair_feature[i].descriptor[k] - map_feature[j].current_descriptor[k])*(pair_feature[i].descriptor[k] - map_feature[j].current_descriptor[k]);
						}

						dist = sqrt(dist);

						if( dist < similar_threshold ) is_similar = true;
						//app_point << "ok4_5" << endl;
					}

					//ix位置
					if (int(pair_feature[i].ix/106)==g) locat=true;

					if( !is_similar && !is_near && !pair_feature[i].match  && locat)
					{
						//app_point << "ok4_6" << endl;
						memset( &temp , 0 , sizeof(Keep_Feature) );
						temp.l_ix = pair_feature[i].ix;
						temp.l_iy = pair_feature[i].iy;
						temp.r_ix = pair_feature[i].r_ix;
						temp.r_iy = pair_feature[i].r_iy;
						temp.hessian = pair_feature[i].hessian;
						temp.laplacian = pair_feature[i].laplacian;
						temp.size	   = pair_feature[i].size;
						temp.dir	   = pair_feature[i].dir;
						temp.match     = true;



						for(int j=0 ; j<16 ; j++)
						{
							temp.original_descriptor[j] =  pair_feature[i].descriptor[j]; 
							//temp.current_descriptor[j] =  pair_feature[i].descriptor[j];
						}

						temp.num = feature_num;
						feature_num++;
						on_image_num++;
						location[g]++;
						double hz=0,hy=0;
						///////////////////SSG
						/*
						//temp.hz = L*((l_fu+r_fu)/2)/( temp.l_ix - temp.r_ix );
						temp.hz = L*l_fu*r_fu/(r_fu*(temp.l_ix-l_u0)-l_fu*(temp.r_ix-r_u0));
						temp.hx = temp.hz*( temp.l_ix - l_u0 )/l_fu;
						temp.hy = temp.hz*( temp.l_iy - l_v0 )/l_fv;

						hz = temp.hz;
						hy = temp.hy;

						temp.hz = -hy;
						temp.hy = hz;*/
						// 特徵點的左攝影機3維座標 (洪_非SSG)
						double gxL = (temp.l_ix - l_u0)/l_fu;
						double gyL = -(temp.l_iy - l_v0)/l_fv;
						double gxR = (temp.r_ix - r_u0)/r_fu;

						double k = ( cvmGet(R_matrix,0,0) - cvmGet(R_matrix,2,0)*gxR )*gxL +( cvmGet(R_matrix,0,1) - cvmGet(R_matrix,2,1)*gxR )*gyL + ( cvmGet(R_matrix,0,2) - cvmGet(R_matrix,2,2)*gxR ) ;
						temp.hz = ( cvmGet(T_matrix,0,2)*gxR - cvmGet(T_matrix,0,0))/(k*1000);
		    			temp.hx = temp.hz*gxL;
						temp.hy = temp.hz*gyL;

						
						hz = temp.hz;
						hy = temp.hy;
						temp.hz = -hy;
						temp.hy = hz;


						map_feature.push_back(temp);
						//app_point << "ok4_7" << endl;
					}
				}
			}

	}
	//
    for (int i=0;i<3;i++)
	{
		app_state<<"location["<<i<<"]" <<setw(2)<<location[i]<<endl;
	}
	app_state<<endl;
	//
	//app_point << "ok4_8" << endl;

	//app_point.close();
	app_state.close();
}


void SLAM::SLAM_Set_Hessian_Threshold(void)
{
	if( on_image_num == 0  &&  hessian_threshold - 1000 >= minimum_hessian_threshold )
	{
		hessian_threshold -= 1000;
	}
	//else if( on_image_num == 0  &&  hessian_threshold - 1000 < minimum_hessian_threshold )
	//{
	//hessian_threshold = minimum_hessian_threshold;
	//}
	else if( on_image_num < max_on_image_num )
	{
		if( hessian_threshold - 50*( max_on_image_num - on_image_num ) >= minimum_hessian_threshold )
		{
			hessian_threshold -= 50*( max_on_image_num - on_image_num );
		}
		//else if ( hessian_threshold - 50*( max_on_image_num - on_image_num ) < minimum_hessian_threshold )
		//{
		//hessian_threshold = minimum_hessian_threshold;
		//}
	}
	else if( on_image_num > max_on_image_num )
	{
		/*
		if( hessian_threshold + 50*( on_image_num - max_on_image_num ) <= maximum_hessian_threshold )
		{
		hessian_threshold += 50*( on_image_num - max_on_image_num );
		}
		*/
		if( hessian_threshold + 50 <= maximum_hessian_threshold )
		{
			hessian_threshold += 50;
		}
	}



}

void SLAM::RANSAC(void)
{	
	//fstream app_point("point.txt",ios::app);
	fstream app_root("root.txt",ios::app);
	fstream app_RRANSACtime("RRANSACtime.txt",ios::app);		
		fstream app_P3Ptime("P3Ptime.txt",ios::app);	
//	fstream app_root("root.txt",ios::app);
	//app_point << "ok4_1_1 RANSAC" << endl;

	bool again = true; 
	int threshold = 3;//last_map_feature.size()*0.6;
	double range = 1.2;
	
	double p[]={l_fu,l_fv,l_u0,l_v0,0,0,0,0};
	//int b=0;
	int maxsetsize=0;
	double time1,time2,time3;
	time3=0;
	double time4,time5,time6;
	time6=0;
	int num=0;
	time4=(double)cvGetTickCount();	
						int numc=0 , numnc=0;
						unsigned seed;
						double RRANSACtime1,RRANSACtime2,RRANSACtime3;
						double P3Ptime1,P3Ptime2,P3Ptime3;
						P3Ptime3=0;
						RRANSACtime3=0;
						seed=(unsigned) time(NULL);
						srand(seed);

	for(int g=0; again && g<last_map_feature.size()-2 ; g++)
	{
		for(int g_two=g+1 ; again && g_two< last_map_feature.size()-1 ; g_two++ )
		{
			for(int g_three=g_two+1 ; again && g_three< last_map_feature.size() ; g_three++ )
			{
				vector<Matrix> X; 
				X.clear();
				vector< Rectangular > Rs;
				Rs.clear();
				vector< Matrix > ts;
				ts.clear();
				
				vector<int> true_match_number;
				true_match_number.clear();

				//app_point << "ok4_1_2" << endl;
				P3PSolver p3p;
	
				double feature[9];
				double worldPoints[9];
				IxIy temp;
				feature[0]=	 (last_map_feature[g].l_ix-l_u0)/l_fu;
				feature[1] =-(last_map_feature[g].l_iy-l_v0)/l_fv;
				feature[2] = 1;
				double na0=sqrt(feature[0]*feature[0]+feature[1]*feature[1]+feature[2]*feature[2]);
				feature[0]=feature[0]/na0;
				feature[1]=feature[1]/na0;
				feature[2]=feature[2]/na0;


				feature[3] =(last_map_feature[g_two].l_ix-l_u0)/l_fu;
				feature[4] =-(last_map_feature[g_two].l_iy-l_v0)/l_fv;
				feature[5] =1;
				double na1=sqrt(feature[3]*feature[3]+feature[4]*feature[4]+feature[5]*feature[5]);
				feature[3]=feature[3]/na1;
				feature[4]=feature[4]/na1;
				feature[5]=feature[5]/na1;


				feature[6] =(last_map_feature[g_three].l_ix-l_u0)/l_fu;
				feature[7] =-(last_map_feature[g_three].l_iy-l_v0)/l_fv;
				feature[8] = 1;
				double na2=sqrt(feature[6]*feature[6]+feature[7]*feature[7]+feature[8]*feature[8]);
				feature[6]=feature[6]/na2;
				feature[7]=feature[7]/na2;
				feature[8]=feature[8]/na2;
					
				//app_point << "ok4_1_3" << endl;
				
				worldPoints[0] = last_map_feature[g].hx;
 				worldPoints[1] = -last_map_feature[g].hz;
				worldPoints[2] = last_map_feature[g].hy;
				worldPoints[3] =  last_map_feature[g_two].hx;
 				worldPoints[4] = -last_map_feature[g_two].hz;
				worldPoints[5]=  last_map_feature[g_two].hy;
				worldPoints[6] =last_map_feature[g_three].hx;
 				worldPoints[7] =-last_map_feature[g_three].hz;
				worldPoints[8]=last_map_feature[g_three].hy;

				//app_point << "ok4_1_3" << endl;
					P3Ptime1=(double)cvGetTickCount();
					
					p3p.computePoses( worldPoints,feature, X, Rs );


					P3Ptime2=(double)cvGetTickCount();
					P3Ptime3=(double)((P3Ptime2-P3Ptime1)/(cvGetTickFrequency()*1000)+P3Ptime3);
				//app_point << "ok4_1_7" << endl;


				int match_number = 0;
				int match_max = 0;

				for(int i=0 ; i<X.size() ; i++)
				{
					app_root<<X[i].data_[0]<<"   "<<X[i].data_[1]<<"   "<<X[i].data_[2]<<endl;
					vector<int> true_number;
					true_number.clear();
					true_number.push_back(g);
					true_number.push_back(g_two);
					true_number.push_back(g_three);
					int match = 0;
				
					double rx = X[i].data_[0];
					double ry = X[i].data_[1];
					double rz = X[i].data_[2];

					//app_point << "ok4_1_8" << endl;

						int setsize=0;
						int first =0;
						RRANSACtime1=(double)cvGetTickCount();
						int  findone=0;
						int remj;
						int remlow=-2;
					for(int j=0 ; j<last_map_feature.size() ; j++)
					{
		int randj,remj;
						if (first == 1 && setsize ==1)
						{
							j=0;
							first=2;
						}

						if (setsize==0 && first==0)
						{	
							randj=rand() % last_map_feature.size();
							
							j=randj;
							remj=j;
							first=1;
						
						}
					//	app_LOW<<randj<<endl;
						if(j==remj && j!=0 &&setsize>=1 )continue;
						if(g == j || g_two == j || g_three == j) 				
						{
							first=0;
							continue;
						}
						memset(&temp , 0 ,sizeof(IxIy));

						double feature[3] , h[3] ;//, h2[3];
						feature[0] = last_map_feature[j].hx - rx;
						feature[1] = last_map_feature[j].hy - ry;
						feature[2] = last_map_feature[j].hz - rz;
			
						h[0] = Rs[i].data_[0][0]*feature[0]+Rs[i].data_[1][0]*feature[1]+Rs[i].data_[2][0]*feature[2];
						h[1] = Rs[i].data_[0][1]*feature[0]+Rs[i].data_[1][1]*feature[1]+Rs[i].data_[2][1]*feature[2];
						h[2] = Rs[i].data_[0][2]*feature[0]+Rs[i].data_[1][2]*feature[1]+Rs[i].data_[2][2]*feature[2];

						temp.data[0] = l_u0 + l_fu*(h[0]/h[2]);
						temp.data[1] = l_v0 + l_fv*(-h[1]/h[2]);
					//	if(( temp.ix > 320 || temp.ix < 0 || temp.iy > 240 || temp.iy < 0)&&setsize==0)break;
						if(( temp.data[0] > 320 || temp.data[0] < 0 || temp.data[1] > 240 || temp.data[1] < 0)&&setsize==0)	break;
						if( temp.data[0] > 320 || temp.data[0] < 0 || temp.data[1] > 240 || temp.data[1] < 0)	continue;

						//double dist1 = range ;
						//int number = 0;
						double dist = 0;
						dist = (temp.data[0] - last_map_feature[j].l_ix)*(temp.data[0] - last_map_feature[j].l_ix)+(temp.data[1] - last_map_feature[j].l_iy)*(temp.data[1] - last_map_feature[j].l_iy);
						dist = sqrt(dist);
						
						if(dist < range )
						{
							true_number.push_back(j);
							setsize++;
						}
					}
					if (setsize>maxsetsize) maxsetsize=setsize; //計算最大集合個數
				;	if(setsize ==0)break;
					//app_point << "ok4_1_9" << endl;

					if(match_max<true_number.size())
					{
						true_match_number.clear();
						true_match_number = true_number;
						match_max = true_match_number.size();
						match_number = i;
						//cout << "match_max   " << match_max << endl;
					}
				}
					RRANSACtime2=(double)cvGetTickCount();
					RRANSACtime3=(double)((RRANSACtime2-RRANSACtime1)/(cvGetTickFrequency()*1000)+RRANSACtime3);
				app_root<<endl;

				if( match_max > threshold )
				{
					Ransac_Pos ransac_temp;

					ransac_temp.match_set = true_match_number;
					for (int p=0;p<3;p++)
					{
						for(int o=0;o<3;o++)
						{
						  ransac_temp.Rs[p][o]=Rs[match_number].data_[p][o];
						}
						ransac_temp.X[p]=X[match_number].data_[p];
					}
					ransac_match.push_back(ransac_temp);
					//again = false;
				}

				/*fstream app_X_size("txt\\X_size.txt",ios::app);
				app_X_size << X.size() << endl;
				app_X_size.close();*/
				
				//delete []ptw;
				//delete []pti;

				X.swap(vector< Matrix >());
				Rs.swap(vector< Rectangular >());
				ts.swap(vector< Matrix >());

			}


		}


	}
	app_P3Ptime<<P3Ptime3<<endl;
	app_RRANSACtime<<RRANSACtime3<<endl;
	vector<Ransac_Pos> ransac_temp1;
    ransac_temp1=ransac_match;
	ransac_match.clear();
	for(int q=0;q<ransac_temp1.size();q++) //只存最大集合
	{
		if(ransac_temp1[q].match_set.size()==maxsetsize+3)ransac_match.push_back(ransac_temp1[q]);
	}

	fstream app_x("position.txt",ios::app);
	app_x<<ransac_time<<"     "<<ransac_match.size()<<"  0"<<"  0"<<"  0"<<"  0"<<"  0"<<endl;
	
	if (ransac_match.size())
	{
	 for (int u=0;u<ransac_match.size();u++)
	 {
		 app_x<<ransac_match[u].match_set.size()<<"    ";
		 app_x<<ransac_match[u].X[0]<<"   "<<ransac_match[u].X[1]<<"    "<<ransac_match[u].X[2]<<"    ";
		 app_x<<atan2(ransac_match[u].Rs[2][1],ransac_match[u].Rs[2][2])*180/PI-90<<"    ";
		 app_x<<atan2(ransac_match[u].Rs[1][0],ransac_match[u].Rs[0][0])*180/PI<<"    ";
		 app_x<<asin(-ransac_match[u].Rs[2][0])*180/PI<<endl;
	 }
	}
	else app_x<<endl;

	//app_point.close();
	app_x.close();
	app_root.close();

	//Output all the probable extrinsic parameters	
}