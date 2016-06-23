//20151118新增
#include "RANSAC.h"

RANSAC::RANSAC()
{

}

RANSAC::~RANSAC()
{

}


void RANSAC::Initial_RANSAC(double u0, double v0, double fu, double fv)
{
	l_u0 = u0;
	l_v0 = v0;
	l_fu = fu;
	l_fv = fv;
}

//----------------------------------------------------------------------------------------------------宣告
//void SelectWorldCoordinates(Keep_Feature &TheFeature);
void AverageOfWorldCoordinates(Keep_Feature &TheFeature, int Num);
//----------------------------------------------------------------------------------------------------


void RANSAC::RandomSampleConsensus(vector<Keep_Feature> &map_feature , vector<Keep_Feature> last_map_feature, vector<Ransac_Pos> &ransac_match , int PhotoCount)
{
	//fstream app_point("point.txt",ios::app);
//	fstream app_root("root.txt", ios::app);
//	fstream app_RRANSACtime("RRANSACtime.txt", ios::app);
//	fstream app_P3Ptime("P3Ptime.txt", ios::app);
//	fstream app_test("TEST.txt", ios::app);
	//app_point << "ok4_1_1 RANSAC" << endl;

	bool again = true;
	int threshold = 3;//last_map_feature.size()*0.6;
	double range = SetDeviation;	//20151222

	//double p[] = { l_fu, l_fv, l_u0, l_v0, 0, 0, 0, 0 };
	//int b=0;
	int maxsetsize = 0;
	double time1, time2, time3;
	time3 = 0;
	double time4, time5, time6;
	time6 = 0;
	int num = 0;
	time4 = (double)cvGetTickCount();
	int numc = 0, numnc = 0;
	unsigned seed;
	double RRANSACtime1, RRANSACtime2, RRANSACtime3;
	double P3Ptime1, P3Ptime2, P3Ptime3;
	P3Ptime3 = 0;
	RRANSACtime3 = 0;
	seed = (unsigned)time(NULL);
	srand(seed);

	int count_P3P_times = 1;
	//app_test << "//----------------------------------------------------------------------------------------------------" << PhotoCount << endl;

	for (int g = 0; again && g<last_map_feature.size() - 2; g++)
	{
		for (int g_two = g + 1; again && g_two< last_map_feature.size() - 1; g_two++)
		{
			for (int g_three = g_two + 1; again && g_three< last_map_feature.size(); g_three++)
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
				feature[0] = (last_map_feature[g].l_ix - l_u0) / l_fu;
				feature[1] = -(last_map_feature[g].l_iy - l_v0) / l_fv;
				feature[2] = 1;
				double na0 = sqrt(feature[0] * feature[0] + feature[1] * feature[1] + feature[2] * feature[2]);
				feature[0] = feature[0] / na0;
				feature[1] = feature[1] / na0;
				feature[2] = feature[2] / na0;


				feature[3] = (last_map_feature[g_two].l_ix - l_u0) / l_fu;
				feature[4] = -(last_map_feature[g_two].l_iy - l_v0) / l_fv;
				feature[5] = 1;
				double na1 = sqrt(feature[3] * feature[3] + feature[4] * feature[4] + feature[5] * feature[5]);
				feature[3] = feature[3] / na1;
				feature[4] = feature[4] / na1;
				feature[5] = feature[5] / na1;


				feature[6] = (last_map_feature[g_three].l_ix - l_u0) / l_fu;
				feature[7] = -(last_map_feature[g_three].l_iy - l_v0) / l_fv;
				feature[8] = 1;
				double na2 = sqrt(feature[6] * feature[6] + feature[7] * feature[7] + feature[8] * feature[8]);
				feature[6] = feature[6] / na2;
				feature[7] = feature[7] / na2;
				feature[8] = feature[8] / na2;

				//app_point << "ok4_1_3" << endl;

				worldPoints[0] = last_map_feature[g].X;
				worldPoints[1] = last_map_feature[g].Z;
				worldPoints[2] = last_map_feature[g].Y;
				worldPoints[3] = last_map_feature[g_two].X;
				worldPoints[4] = last_map_feature[g_two].Z;
				worldPoints[5] = last_map_feature[g_two].Y;
				worldPoints[6] = last_map_feature[g_three].X;
				worldPoints[7] = last_map_feature[g_three].Z;
				worldPoints[8] = last_map_feature[g_three].Y;


				//app_point << "ok4_1_3" << endl;
				P3Ptime1 = (double)cvGetTickCount();


				p3p.computePoses(worldPoints, feature, X, Rs);




				P3Ptime2 = (double)cvGetTickCount();
				//P3Ptime3 = (double)((P3Ptime2 - P3Ptime1) / (cvGetTickFrequency() * 1000));	//單次
				//app_P3Ptime << P3Ptime3 << endl;
				P3Ptime3 = (double)((P3Ptime2 - P3Ptime1) / (cvGetTickFrequency() * 1000) + P3Ptime3);
				//app_point << "ok4_1_7" << endl;


				int match_number = 0;
				int match_max = 0;
				double match_dist;


				for (int i = 0; i<X.size(); i++)
				{
					//app_test << setw(15) << X[i].data_[0] << setw(15) << X[i].data_[2] << setw(15) << X[i].data_[1] << endl;
					vector<int> true_number;
					true_number.clear();

					double rx = X[i].data_[0];
					double ry = X[i].data_[1];
					double rz = X[i].data_[2];

					//app_point << "ok4_1_8" << endl;

					int setsize = 0;
					double temp_dist = 0;
					RRANSACtime1 = (double)cvGetTickCount();
//----------------------------------------------------------------------------------------------------修正旋轉矩陣
					//double temp_sitaY = asin(-Rs[i].data_[2][0]);
					//Rs[i].data_[0][0] = cos(temp_sitaY);
					//Rs[i].data_[0][1] = 0;
					//Rs[i].data_[0][2] = sin(temp_sitaY);

					//Rs[i].data_[1][0] = 0;
					//Rs[i].data_[1][1] = 1;
					//Rs[i].data_[1][2] = 0;

					//Rs[i].data_[2][0] = -sin(temp_sitaY);
					//Rs[i].data_[2][1] = 0;
					//Rs[i].data_[2][2] = cos(temp_sitaY);
//----------------------------------------------------------------------------------------------------
					for (int j = 0; j<last_map_feature.size(); j++)
					{
						//20151222
						double XX, YY, ZZ;
						XX = Rs[i].data_[0][0] * last_map_feature[j].hx
							+ Rs[i].data_[0][1] * (-last_map_feature[j].hz)
							+ Rs[i].data_[0][2] * last_map_feature[j].hy
							+ rx
							- last_map_feature[j].X;
						ZZ = Rs[i].data_[1][0] * last_map_feature[j].hx
							+ Rs[i].data_[1][1] * (-last_map_feature[j].hz)
							+ Rs[i].data_[1][2] * last_map_feature[j].hy
							+ ry
							- last_map_feature[j].Z;
						YY = Rs[i].data_[2][0] * last_map_feature[j].hx
							+ Rs[i].data_[2][1] * (-last_map_feature[j].hz)
							+ Rs[i].data_[2][2] * last_map_feature[j].hy
							+ rz
							- last_map_feature[j].Y;

						double dist = sqrt(XX * XX + YY * YY + ZZ * ZZ);

//----------------------------------------------------------------------------------------------------強制加入建模點
						//if (j == g || j == g_two || j == g_three)	//20151228
						//{
						//	true_number.push_back(j);
						//	setsize++;
						//	temp_dist += dist;
						//}
						//else
						{
							if (dist < range)
							{
								true_number.push_back(j);
								setsize++;
								temp_dist += dist;
							}
						}
//----------------------------------------------------------------------------------------------------
					}
					//app_test << temp_dist << endl;
					//for (int o = 0; o < true_number.size(); o++)
					//{
					//	app_test << true_number[o] << "    ";
					//}
					//app_test << endl<< endl;


					if (setsize > maxsetsize)	maxsetsize = setsize; //計算最大集合個數
						
					//if (setsize == 0)break;
					//app_point << "ok4_1_9" << endl;

					if (match_max<true_number.size())
					{
						true_match_number.clear();
						true_match_number = true_number;
						match_max = true_match_number.size();
						match_number = i;
						match_dist = temp_dist;
						//cout << "match_max   " << match_max << endl;
					}
				}
				RRANSACtime2 = (double)cvGetTickCount();
				RRANSACtime3 = (double)((RRANSACtime2 - RRANSACtime1) / (cvGetTickFrequency() * 1000) + RRANSACtime3);

				//app_test << endl;

				if (match_max > threshold)
				{
					Ransac_Pos ransac_temp;

					ransac_temp.match_set = true_match_number;
					for (int p = 0; p<3; p++)
					{
						for (int o = 0; o<3; o++)
						{
							ransac_temp.Rs[p][o] = Rs[match_number].data_[p][o];
						}
						ransac_temp.X[p] = X[match_number].data_[p];
					}
					ransac_temp.model[0][0] = g;
					ransac_temp.model[0][1] = last_map_feature[g].num;
					ransac_temp.model[1][0] = g_two;
					ransac_temp.model[1][1] = last_map_feature[g_two].num;
					ransac_temp.model[2][0] = g_three;
					ransac_temp.model[2][1] = last_map_feature[g_three].num;

					ransac_temp.count_time = P3Ptime3;
					//ransac_temp.count_P3P_times = count_P3P_times;

					ransac_temp.X_dist = match_dist;
					ransac_match.push_back(ransac_temp);
					//again = false;
				}


				X.swap(vector< Matrix >());
				Rs.swap(vector< Rectangular >());
				ts.swap(vector< Matrix >());

				count_P3P_times++;
			}
		}
	}
//	app_P3Ptime << P3Ptime3 << endl;
//	app_RRANSACtime << RRANSACtime3 << endl;
	vector<Ransac_Pos> ransac_temp1;
	ransac_temp1 = ransac_match;
	ransac_match.clear();



	//20151222改
	double NN = 1000000;
	double MM = 1000000;
	for (int q = 0; q < ransac_temp1.size(); q++)
	{
//----------------------------------------------------------------------------------------------------最大集合
		//if (ransac_temp1[q].match_set.size() >= (0.85*last_map_feature.size()))
		if (ransac_temp1[q].match_set.size() == maxsetsize)
		{
//----------------------------------------------------------------------------------------------------輸出內群點
			//for (int j = 0; j < ransac_temp1[q].match_set.size(); j++)
			//{
			//	int i = ransac_temp1[q].match_set[j];
			//	//app_RRANSACtime << i << "  ";
			//}

			//app_RRANSACtime << endl << endl;
//----------------------------------------------------------------------------------------------------總誤差最小
			//if ((ransac_temp1[q].X_dist / ransac_temp1[q].match_set.size()) < MM)
			if (ransac_temp1[q].X_dist < NN)
			{
				//app_RRANSACtime << endl << endl;
				NN = ransac_temp1[q].X_dist;
				MM = ransac_temp1[q].X_dist / ransac_temp1[q].match_set.size();
				ransac_match.insert(ransac_match.begin(), ransac_temp1[q]);
			}
		}
	}

	//app_RRANSACtime << PhotoCount << "------------------------------------------------------------" << endl << endl;
//----------------------------------------------------------------------------------------------------輸出模型編號
	//app_RRANSACtime
	//	<< setw(4) << ransac_match[0].model[0][0] << setw(8) << ransac_match[0].model[0][1] << endl
	//	<< setw(4) << ransac_match[0].model[1][0] << setw(8) << ransac_match[0].model[1][1] << endl
	//	<< setw(4) << ransac_match[0].model[2][0] << setw(8) << ransac_match[0].model[2][1]
	//	<< setw(15) << ransac_match[0].count_time << endl << endl;
	////app_test << setw(8) << ransac_match[0].model[0][1] << endl
	////	<< setw(8) << ransac_match[0].model[1][1] << endl
	////	<< setw(8) << ransac_match[0].model[2][1] << endl << endl;
//----------------------------------------------------------------------------------------------------輸出內群點編號
	//for (int j = 0; j < ransac_match[0].match_set.size(); j++)
	//{
	//	int i = ransac_match[0].match_set[j];
	//	app_RRANSACtime << last_map_feature[i].num << "  ";
	//}
	//app_RRANSACtime << endl << endl;
//----------------------------------------------------------------------------------------------------輸出總誤差和平均誤差
	//app_RRANSACtime
	//	<< setw(15) << ransac_match[0].X_dist
	//	<< setw(15) << ransac_match[0].X_dist / ransac_match[0].match_set.size() << endl << endl;
//----------------------------------------------------------------------------------------------------輸出位置和旋轉矩陣
	//app_RRANSACtime
	//	<< setw(15) << ransac_match[0].X[0]
	//	<< setw(15) << ransac_match[0].X[1]
	//	<< setw(15) << ransac_match[0].X[2] << endl << endl;
	//app_RRANSACtime
	//	<< setw(15) << ransac_match[0].Rs[0][0]
	//	<< setw(15) << ransac_match[0].Rs[0][1]
	//	<< setw(15) << ransac_match[0].Rs[0][2] << endl
	//	<< setw(15) << ransac_match[0].Rs[1][0]
	//	<< setw(15) << ransac_match[0].Rs[1][1]
	//	<< setw(15) << ransac_match[0].Rs[1][2] << endl
	//	<< setw(15) << ransac_match[0].Rs[2][0]
	//	<< setw(15) << ransac_match[0].Rs[2][1]
	//	<< setw(15) << ransac_match[0].Rs[2][2] << endl;

	//app_RRANSACtime << endl;
//----------------------------------------------------------------------------------------------------只留一個
	Ransac_Pos temp_ransac_match;
	temp_ransac_match = ransac_match[0];
	ransac_match.clear();
	ransac_match.push_back(temp_ransac_match);
//----------------------------------------------------------------------------------------------------
	//20151222
//	app_root << PhotoCount << "------------------------------------------------------------" << endl;
	for (int i = 0; i < map_feature.size(); i++)
	{
		if (map_feature[i].match == 1)
		{
//			app_root << setw(4) << map_feature[i].num << endl;
//----------------------------------------------------------------------------------------------------紀錄世界座標位置
			World_Coordinates temp_Xw;

			temp_Xw.photonum = PhotoCount;
			temp_Xw.X = ransac_match[0].Rs[0][0] * map_feature[i].hx
				+ ransac_match[0].Rs[0][1] * (-map_feature[i].hz)
				+ ransac_match[0].Rs[0][2] * map_feature[i].hy
				+ ransac_match[0].X[0];
			temp_Xw.Z = ransac_match[0].Rs[1][0] * map_feature[i].hx
				+ ransac_match[0].Rs[1][1] * (-map_feature[i].hz)
				+ ransac_match[0].Rs[1][2] * map_feature[i].hy
				+ ransac_match[0].X[1];
			temp_Xw.Y = ransac_match[0].Rs[2][0] * map_feature[i].hx
				+ ransac_match[0].Rs[2][1] * (-map_feature[i].hz)
				+ ransac_match[0].Rs[2][2] * map_feature[i].hy
				+ ransac_match[0].X[2];

			temp_Xw.accurate = true;	//20160124

			map_feature[i].Xw.push_back(temp_Xw);
//----------------------------------------------------------------------------------------------------世界座標平均位置
			double temp_total_X = 0, temp_total_Y = 0, temp_total_Z = 0;
			int temp_N = 0;

			for (int j = 0; j < map_feature[i].Xw.size(); j++)
			{
				temp_total_X += map_feature[i].Xw[j].X;
				temp_total_Y += map_feature[i].Xw[j].Y;
				temp_total_Z += map_feature[i].Xw[j].Z;
				//app_root << setw(8) << map_feature[i].Xw[j].photonum;	//舊的
				//app_root << setw(15) << map_feature[i].Xw[j].X;
				//app_root << setw(15) << map_feature[i].Xw[j].Y;
				//app_root << setw(15) << map_feature[i].Xw[j].Z;
				//app_root << setw(4) << map_feature[i].Xw[j].accurate << endl;
			}
			map_feature[i].X = temp_total_X / map_feature[i].Xw.size();
			map_feature[i].Y = temp_total_Y / map_feature[i].Xw.size();
			map_feature[i].Z = temp_total_Z / map_feature[i].Xw.size();
			//app_root << setw(23) << map_feature[i].X;
			//app_root << setw(15) << map_feature[i].Y;
			//app_root << setw(15) << map_feature[i].Z << endl;
//----------------------------------------------------------------------------------------------------標準差
			double temp_dist = 0;
			for (int j = 0; j < map_feature[i].Xw.size(); j++)
			{
				map_feature[i].Xw[j].dist = (map_feature[i].Xw[j].X - map_feature[i].X)
													* (map_feature[i].Xw[j].X - map_feature[i].X)
													+ (map_feature[i].Xw[j].Y - map_feature[i].Y)
													* (map_feature[i].Xw[j].Y - map_feature[i].Y)
													+ (map_feature[i].Xw[j].Z - map_feature[i].Z)
													* (map_feature[i].Xw[j].Z - map_feature[i].Z);	//20160124

				temp_dist += map_feature[i].Xw[j].dist;

				map_feature[i].Xw[j].dist = sqrt(map_feature[i].Xw[j].dist);

				map_feature[i].Xw[j].accurate = true;

// 				app_root << setw(8) << map_feature[i].Xw[j].photonum;
// 				app_root << setw(15) << map_feature[i].Xw[j].X;
// 				app_root << setw(15) << map_feature[i].Xw[j].Y;
// 				app_root << setw(15) << map_feature[i].Xw[j].Z;
// 				app_root << setw(20) << map_feature[i].Xw[j].dist;
// 				app_root << setw(4) << map_feature[i].Xw[j].accurate << endl;
			}
			map_feature[i].deviation = sqrt(temp_dist / map_feature[i].Xw.size());

// 			app_root << endl;
// 			app_root << setw(23) << map_feature[i].X;
// 			app_root << setw(15) << map_feature[i].Y;
// 			app_root << setw(15) << map_feature[i].Z << endl << endl;
// 			app_root << setw(15) << map_feature[i].deviation << endl << endl;
//----------------------------------------------------------------------------------------------------標記不好的特徵點，篩選世界座標位置
			if (map_feature[i].deviation > SetDeviation)	//20160124
			{
				map_feature[i].GoodorBad = false;
				if (map_feature[i].Xw.size() > 2)
				{
					SelectWorldCoordinates(map_feature[i]);
				}
			}
//----------------------------------------------------------------------------------------------------
		}
	}
//----------------------------------------------------------------------------------------------------刪除特徵點
	//vector<Keep_Feature> temp_map_feature;
	//temp_map_feature = map_feature;
	//map_feature.clear();

	//for (int k = 0; k < temp_map_feature.size(); k++)
	//{
	//	if (temp_map_feature[k].deviation < 0.01)	//1cm
	//	{
	//		map_feature.push_back(temp_map_feature[k]);
	//	}
	//}
//----------------------------------------------------------------------------------------------------標記不好的特徵點
	//for (int k = 0; k < map_feature.size(); k++)
	//{
	//	if (map_feature[k].GoodorBad == 1 && map_feature[k].deviation > 0.008)
	//	{
	//		map_feature[k].GoodorBad = false;
	//	}
	//}
//----------------------------------------------------------------------------------------------------畫建模的特徵點(照片)
	//for (int i = 0; i < map_feature.size(); i++)
	//{
	//	map_feature[i].match = false;
	//	for (int j = 0; j < 3; j++)
	//	{
	//		if (map_feature[i].num == ransac_match[0].model[j][1])
	//		{
	//			map_feature[i].match = true;
	//		}
	//	}
	//}


//----------------------------------------------------------------------------------------------------更新20151208
	//app_root << PhotoCount << endl;
	//for (int i = 0; i < map_feature.size(); i++)
	//{
		//map_feature[i].update = true;
		//for (int j = 0; j < ransac_match[0].match_set.size(); j++)
		//{
		//	if (map_feature[i].num == last_map_feature[ransac_match[0].match_set[j]].num)
		//	{
		//		//last_map_feature[ransac_match[0].match_set[j]].match = 0;
		//		map_feature[i].update= false;
		//		//app_root << last_map_feature[ransac_match[0].match_set[j]].match << "     ";
		//		app_root << map_feature[i].match << "     ";
		//		app_root << map_feature[i].num << "     " << last_map_feature[ransac_match[0].match_set[j]].num << endl;
		//		
		//		break;
		//	}
		//}

		//if (map_feature[i].update == 1 && map_feature[i].match ==1 && map_feature[i].X == 0 && map_feature[i].Y == 0 && map_feature[i].Z == 0)
		//if (map_feature[i].match == 1)
		//{
			//app_root << "Update     " << map_feature[i].num << endl;

	//		map_feature[i].X = ransac_match[0].Rs[0][0] * map_feature[i].hx
	//			+ ransac_match[0].Rs[0][1] * (-map_feature[i].hz) 
	//			+ ransac_match[0].Rs[0][2] * map_feature[i].hy 
	//			+ ransac_match[0].X[0];
	//		map_feature[i].Z = ransac_match[0].Rs[1][0] * map_feature[i].hx 
	//			+ ransac_match[0].Rs[1][1] * (-map_feature[i].hz) 
	//			+ ransac_match[0].Rs[1][2] * map_feature[i].hy 
	//			+ ransac_match[0].X[1];
	//		map_feature[i].Y = ransac_match[0].Rs[2][0] * map_feature[i].hx 
	//			+ ransac_match[0].Rs[2][1] * (-map_feature[i].hz) 
	//			+ ransac_match[0].Rs[2][2] * map_feature[i].hy 
	//			+ ransac_match[0].X[2];
	//	}
	//}
	//app_root << endl;



	//20151123
	//time2 = (double)(((double)cvGetTickCount() - time4) / (cvGetTickFrequency() * 1000));
	//app_RRANSACtime << time2 << "    " << P3Ptime3 << endl;

	//app_point.close();
	//app_root.close();

	//Output all the probable extrinsic parameters	
}


void RANSAC::SelectWorldCoordinates(Keep_Feature &TheFeature)
{
	RANSAC ransac;
	for (int n = 0; n < TheFeature.Xw.size()-2; n++)
	{
		double MaxDist = 0;
//----------------------------------------------------------------------------------------------------找最大距離
		for (int i = 0; i < TheFeature.Xw.size(); i++)
		{
			if (TheFeature.Xw[i].dist > MaxDist)
			{
				MaxDist = TheFeature.Xw[i].dist;
			}
		}
//----------------------------------------------------------------------------------------------------標記最大距離的座標
		for (int i = 0; i < TheFeature.Xw.size(); i++)
		{
			if (TheFeature.Xw[i].dist == MaxDist)
			{
				TheFeature.Xw[i].accurate = false;
			}
		}
//----------------------------------------------------------------------------------------------------篩選世界座標位置
		AverageOfWorldCoordinates(TheFeature, TheFeature.Xw.size() - n - 1);
//----------------------------------------------------------------------------------------------------標準差到達範圍內跳出
		if (TheFeature.deviation < ransac.SetDeviation)
		{
			TheFeature.GoodorBad = true;
			break;
		}
	}
}

void AverageOfWorldCoordinates(Keep_Feature &TheFeature , int Num)
{
//	fstream app_root("root.txt", ios::app);
//	app_root << "--------------------Update--------------------" << endl << endl;
//----------------------------------------------------------------------------------------------------世界座標平均位置
	double temp_total_X = 0, temp_total_Y = 0, temp_total_Z = 0;

	for (int i = 0; i < TheFeature.Xw.size(); i++)
	{
		if (TheFeature.Xw[i].accurate == 1)
		{
			temp_total_X += TheFeature.Xw[i].X;
			temp_total_Y += TheFeature.Xw[i].Y;
			temp_total_Z += TheFeature.Xw[i].Z;
		}		
	}
	TheFeature.X = temp_total_X / Num;
	TheFeature.Y = temp_total_Y / Num;
	TheFeature.Z = temp_total_Z / Num;
//----------------------------------------------------------------------------------------------------標準差
	double temp_dist = 0;
	for (int i = 0; i < TheFeature.Xw.size(); i++)
	{
		if (TheFeature.Xw[i].accurate == 1)
		{
			TheFeature.Xw[i].dist = (TheFeature.Xw[i].X - TheFeature.X) * (TheFeature.Xw[i].X - TheFeature.X)
											+ (TheFeature.Xw[i].Y - TheFeature.Y) * (TheFeature.Xw[i].Y - TheFeature.Y)
											+ (TheFeature.Xw[i].Z - TheFeature.Z) * (TheFeature.Xw[i].Z - TheFeature.Z);	//20160124

			temp_dist += TheFeature.Xw[i].dist;

			TheFeature.Xw[i].dist = sqrt(TheFeature.Xw[i].dist);
		}
		else
		{
			TheFeature.Xw[i].dist = 0;
		}

// 		app_root << setw(8) << TheFeature.Xw[i].photonum;
// 		app_root << setw(15) << TheFeature.Xw[i].X;
// 		app_root << setw(15) << TheFeature.Xw[i].Y;
// 		app_root << setw(15) << TheFeature.Xw[i].Z;
// 		app_root << setw(20) << TheFeature.Xw[i].dist;
// 		app_root << setw(4) << TheFeature.Xw[i].accurate << endl;
	}

	TheFeature.deviation = sqrt(temp_dist / Num);

// 	app_root << endl;
// 	app_root << setw(23) << TheFeature.X;
// 	app_root << setw(15) << TheFeature.Y;
// 	app_root << setw(15) << TheFeature.Z << endl << endl;
// 	app_root << setw(15) << TheFeature.deviation << endl << endl;
}

//20151207加
void RANSAC::UpdateWorldCoordinates(Ransac_Pos ransac_match , vector<Keep_Feature> &map_feature)
{
	for (int i = 0; i < map_feature.size(); i++)
	{
		if (map_feature[i].match == 0)	continue;	//有比對成功的才更新

		map_feature[i].X = ransac_match.Rs[0][0] * map_feature[i].hx + ransac_match.Rs[0][1] * (-map_feature[i].hz) + ransac_match.Rs[0][2] * map_feature[i].hy + ransac_match.X[0];
		map_feature[i].Z = ransac_match.Rs[1][0] * map_feature[i].hx + ransac_match.Rs[1][1] * (-map_feature[i].hz) + ransac_match.Rs[1][2] * map_feature[i].hy + ransac_match.X[1];
		map_feature[i].Y = ransac_match.Rs[2][0] * map_feature[i].hx + ransac_match.Rs[2][1] * (-map_feature[i].hz) + ransac_match.Rs[2][2] * map_feature[i].hy + ransac_match.X[2];

	}
}