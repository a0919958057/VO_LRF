// SHHuangDlg.h : 標頭檔
//

#pragma once


#include "..//RoboticsPlatform//VideoCapture//DxCapture.h"
#include "..//SLAM//SLAM.h"
#include "afxwin.h"
#include <direct.h>
#include <fstream>
#include <iomanip>
#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <vector>
#include <highgui.h> //CImage 貼圖
#include <direct.h> //_getcwd 路徑
#include "RSocket.h"
//*****************************************************************************************多媒體計時器
#include <windows.h>
#include <mmsystem.h>
#include "afxcmn.h"
#pragma comment( lib, "winmm.lib" )
//*****************************************************************************************

using namespace std;

// CSHHuangDlg 對話方塊
class SHHuangDlg : public CDialog
{
// 建構
public:
	SHHuangDlg(CWnd* pParent = NULL);	// 標準建構函式

// 對話方塊資料
	enum { IDD = IDD_SHHuang_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支援


// 程式碼實作
protected:
	HICON m_hIcon;

	// 產生的訊息對應函式
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	enum TCPEvent {
		CREATE_SOCKET_SUCCESSFUL,
		CREATE_SOCKET_FAIL,
		CONNECT_SUCCESSFUL,
		CONNECT_FAIL,
		DISCONNECT,
		SEND_MESSAGE_SUCCESSFUL,
		SENT_MESSAGE_FAIL
	};

	CRSocket m_socket;
	template<size_t LENGTH> void SendSocketMessage(char(&data)[LENGTH]);

private:
	void DoSocketConnect();
	void DoSocketDisconnect();
	void SHHuangDlg::ReportSocketStatus(TCPEvent event_, CString &msg = CString(""));
	bool fg_connected;

	static bool check_updatedata;
public:
	CComboBox m_L_SelectCamera_c;
	CComboBox m_R_SelectCamera_c;
	afx_msg void OnBnClickedLInitialccd();
	afx_msg void OnBnClickedLOptionccd();
	afx_msg void OnBnClickedLCloseccd();
	afx_msg void OnBnClickedRInitialccd();
	afx_msg void OnBnClickedROptionccd();
	afx_msg void OnBnClickedRCloseccd();
	CStatic m_L_Image_Live_c;
	CStatic m_R_Image_Live_c;
	CStatic m_L_Image_SURF_c;
	CStatic m_R_Image_SURF_c;
	CButton m_SaveImage_Mode_c;
	CButton m_OnLine_Mode_c;
	CButton m_Start_c;
	CButton m_Pause_c;
	CButton m_Stop_c;
	CButton m_Continue_c;
	afx_msg void OnBnClickedStart();
	CButton m_LoadImage_Mode_c;
	CStatic m_Image_PhotoCount_c;
	CStatic m_Map_c;

	double m_SampleTime;
	int m_Frequency;
	int m_Frequency2;
	UINT m_socket_port;
	int m_Time1;
	int m_Time2;
	int m_Time3;
	int m_Time4;
	double m_cameraSitaX;
	double m_cameraSitaY;
	double m_cameraSitaZ;
	double m_cameraX;
	double m_cameraY;
	double m_cameraZ;

	double time1;
	double time2;
	double time3;
	double time4;
	double time5;
	double time6;
	double time7;
	double time8;
	double time9;


	void DoEvent();
	void DeletePhoto();
    void DeletePhoto2();
	void WORK( const IplImage* l_image, const IplImage* r_image, const double SampleTime );
	void ShowImage( const IplImage* image, CWnd* pWnd );
	void save_car_pos();

	static double scale;
	int show_image_num;
    int show_map_num;
	int show_feature_region;
	int show_Search_Window_Size;
	int show_all_feature;
	int txt;


	void ShowPhotoCount( const int PhotoCount, CWnd* pWnd );
	afx_msg void OnBnClickedPause();
	afx_msg void OnBnClickedStop();
	afx_msg void OnBnClickedContinue();

	
	CRect rect_map;
	CWnd* pWnd_map;

	int PhotoCount;

	int move_time; //外
	int delay;
	int active_times;//裡
	int move_active[8];
	int move_turn;
	int p3dxdis;

	double SampleTime;
	vector<double> SampleTime_temp;
	static vector<Keep_Feature> draw_feature;
	static vector<CvPoint2D64f> Path;
	static vector <vector<Keep_Feature>> feature_path;
	static bool carFLAG;
	static void I90_PWM_send(int L_PWM, int R_PWM);
	static unsigned char checksun(int nStar, int nEnd);
	static CvPoint2D64f orgin;

	SLAM BinocularSLAM;
	
	MMRESULT FTimerID; // 多媒體計時器

	bool L_InitialCCD;
	bool R_InitialCCD;


	CButton m_L_InitialCCD_c;
	CButton m_R_InitialCCD_c;
	CButton m_L_OptionCCD_c;
	CButton m_R_OptionCCD_c;
	CButton m_L_CloseCCD_c;
	CButton m_R_CloseCCD_c;
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButtonbig();
	afx_msg void OnBnClickedButtonsmall();
	afx_msg void OnBnClickedButtonUp();
	afx_msg void OnBnClickedButtonDown();
	afx_msg void OnBnClickedButtonLeft();
	afx_msg void OnBnClickedButtonRight();
	afx_msg void OnBnClickedButtonReset();

	//20160131
	int m_FeatureNum;
	double m_X_shift;
	double m_Y_shift;
	double m_Z_shift;
	CButton m_Update;
	afx_msg void OnBnClickedButtonReadfeature();
	static double Camera[6];
	static double map_move[4]; //移動小地圖用
	static double initial_scale;
	static double vr, vl;    //要控制I90到一定位時所用
	static double vr_draw, vl_draw;
	static double car_x, car_y, car_zdir;
	static double target_pos[3];
	static int sampleTime;
	static int state;
	//---------thread car_draw------------
private:
	CWinThread * m_pThread_car_draw;
	bool Continue_car_draw;
	static UINT ThreadFun_car_draw(LPVOID lParam);
	//---------thread TARGET_control------------
private:
	CWinThread * m_pThread_TARGET_control;
	bool Continue_TARGET_control;
	static UINT ThreadFun_TARGET_control(LPVOID lParam);

public:
	afx_msg void OnBnClickedSocketConnect();
	afx_msg void OnBnClickedButtonConnectI90();
	BOOL PreTranslateMessage(MSG* pMsg);
	CIPAddressCtrl m_socket_ip_c;
	CListBox m_socket_log_c;
	CButton m_socket_connect_c;
	CButton m_read_map_c;
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	CButton m_FollowMode_c;
	CButton m_LoadImage_Mode_Laser_c;
};
