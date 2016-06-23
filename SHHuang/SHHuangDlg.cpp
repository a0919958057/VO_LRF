
// YJHuangDlg.cpp : 實作檔
//

#include "stdafx.h"
#include "SHHuang.h"
#include "SHHuangDlg.h"
#include "CvvImage.h"
#include "Serial.h"

using namespace std;

#define WNU_THREAD_EXIT (WM_USER + 1)

VideoCapture L_Cam;
VideoCapture R_Cam;
CSerial Connect_I90;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// 對 App About 使用 CAboutDlg 對話方塊
bool SHHuangDlg::carFLAG = true;
bool SHHuangDlg::check_updatedata = true;
vector<CvPoint2D64f> SHHuangDlg::Path;
vector<Keep_Feature> SHHuangDlg::draw_feature;
vector <vector<Keep_Feature>> SHHuangDlg::feature_path;
double SHHuangDlg::scale;
double SHHuangDlg::initial_scale;
double  SHHuangDlg::Camera[6] = { 0 };
double SHHuangDlg::map_move[4] = { 0 };
double  SHHuangDlg::target_pos[3] = { 0 };
double SHHuangDlg::vr = 0, SHHuangDlg::vl = 0;
double SHHuangDlg::car_x = 0, SHHuangDlg::car_y = 0, SHHuangDlg::car_zdir = 0;
double  SHHuangDlg::vr_draw = 0, SHHuangDlg::vl_draw = 0;
CvPoint2D64f SHHuangDlg::orgin;
double CRSocket::Laser_pos[SIZE_POSE_DATA] = { 0 };
int SHHuangDlg::sampleTime = 0;
int SHHuangDlg::state = 1;
int c_Synchronous = 160;  //I90補正
double Magnification = 1.2; //放大倍率
int sleep_time = 0;
CPoint mouse_pos;

char I90_PWM_control[15] = {
	94,    //0  STX  0x5e|0x02
	2,		//1  STX
	1,		//2  RID
	0,		//3  Reserved
	5,		//4  DID  5的話是各輪子PWM控制
	6,		 //5  Length
	1,		//6   選擇輪子1(右輪)
	160,		//7   Low_8bit
	15,	//8   High_8bit(64就是16384，為中間值，右輪比16384小為前進)
	0,		 //9  選擇輪子0(左輪)
	160,		//10  Low_8bit
	15,	//11  High_8bit(64就是16384，為中間值，左輪比16384大為前進)
	0,		//12  Checksum
	94,	//13  ETX 0x5E|0X0D
	13 }; //14

struct THREAD_INFO_car_draw
{
	bool * Continue;//是否繼續執行
	HWND hWnd;//產生執行續的視窗物件
}Thread_Info_car_draw;

struct THREAD_INFO_TARGET_control
{
	bool * Continue;//是否繼續執行
	HWND hWnd;//產生執行續的視窗物件
}Thread_Info_TARGET_control;

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

	// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

	// 程式碼實作
protected:
	DECLARE_MESSAGE_MAP()
};

CRSocket::CRSocket() {}

CRSocket::~CRSocket() {}

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// CYJHuangDlg 對話方塊

SHHuangDlg::SHHuangDlg(CWnd* pParent /*=NULL*/)
	: CDialog(SHHuangDlg::IDD, pParent)
	, m_SampleTime(0)
	, m_Time1(0)
	, m_Time2(0)
	, m_Time3(0)
	, m_Time4(0)
	, m_cameraSitaX(0)
	, m_cameraSitaY(0)
	, m_cameraSitaZ(0)
	, m_cameraX(0)
	, m_cameraY(0)
	, m_cameraZ(0)
	, m_Frequency(0)
	, m_Frequency2(0)
	, m_FeatureNum(0)
	, m_X_shift(0)
	, m_Y_shift(0)
	, m_Z_shift(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void SHHuangDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_L_SelectCamera, m_L_SelectCamera_c);
	DDX_Control(pDX, IDC_R_SelectCamera, m_R_SelectCamera_c);
	DDX_Control(pDX, IDC_L_Image_Live, m_L_Image_Live_c);
	DDX_Control(pDX, IDC_R_Image_Live, m_R_Image_Live_c);
	DDX_Control(pDX, IDC_L_Image_SURF, m_L_Image_SURF_c);
	DDX_Control(pDX, IDC_R_Image_SURF, m_R_Image_SURF_c);
	DDX_Control(pDX, IDC_SaveImage_Mode, m_SaveImage_Mode_c);
	DDX_Control(pDX, IDC_Start, m_Start_c);
	DDX_Control(pDX, IDC_Pause, m_Pause_c);
	DDX_Control(pDX, IDC_Stop, m_Stop_c);
	DDX_Control(pDX, IDC_Continue, m_Continue_c);
	DDX_Control(pDX, IDC_LoadImage_Mode, m_LoadImage_Mode_c);
	DDX_Text(pDX, IDC_SampleTime, m_SampleTime);
	DDX_Text(pDX, IDC_Frequency, m_Frequency);
	DDX_Text(pDX, IDC_Frequency2, m_Frequency2);
	DDX_Text(pDX, IDC_Time1, m_Time1);
	DDX_Text(pDX, IDC_Time2, m_Time2);
	DDX_Text(pDX, IDC_Time3, m_Time3);
	DDX_Text(pDX, IDC_Time4, m_Time4);
	DDX_Text(pDX, IDC_Time5, m_cameraSitaX);
	DDX_Text(pDX, IDC_Time6, m_cameraSitaY);
	DDX_Text(pDX, IDC_Time7, m_cameraSitaZ);
	DDX_Text(pDX, IDC_Time8, m_cameraX);
	DDX_Text(pDX, IDC_Time9, m_cameraY);
	DDX_Text(pDX, IDC_Time10, m_cameraZ);
	DDX_Control(pDX, IDC_Image_PhotoCount, m_Image_PhotoCount_c);
	DDX_Control(pDX, IDC_Map, m_Map_c);
	DDX_Control(pDX, IDC_OnLine_Mode, m_OnLine_Mode_c);
	DDX_Control(pDX, IDC_L_InitialCCD, m_L_InitialCCD_c);
	DDX_Control(pDX, IDC_R_InitialCCD, m_R_InitialCCD_c);
	DDX_Control(pDX, IDC_L_OptionCCD, m_L_OptionCCD_c);
	DDX_Control(pDX, IDC_R_OptionCCD, m_R_OptionCCD_c);
	DDX_Control(pDX, IDC_L_CloseCCD, m_L_CloseCCD_c);
	DDX_Control(pDX, IDC_R_CloseCCD, m_R_CloseCCD_c);
	DDX_Control(pDX, IDC_SOCKET_IP, m_socket_ip_c);
	DDX_Control(pDX, IDC_SOCKET_LOG, m_socket_log_c);
	DDX_Control(pDX, IDC_SOCKET_CONNECT, m_socket_connect_c);
	DDX_Control(pDX, IDC_BUTTON_ReadFeature, m_read_map_c);
	DDX_Control(pDX, IDC_CHECK_Follow, m_FollowMode_c);
	DDX_Control(pDX, IDC_LoadImage_Mode_Laser, m_LoadImage_Mode_Laser_c);
}

BEGIN_MESSAGE_MAP(SHHuangDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_L_InitialCCD, &SHHuangDlg::OnBnClickedLInitialccd)
	ON_BN_CLICKED(IDC_L_OptionCCD, &SHHuangDlg::OnBnClickedLOptionccd)
	ON_BN_CLICKED(IDC_L_CloseCCD, &SHHuangDlg::OnBnClickedLCloseccd)
	ON_BN_CLICKED(IDC_R_InitialCCD, &SHHuangDlg::OnBnClickedRInitialccd)
	ON_BN_CLICKED(IDC_R_OptionCCD, &SHHuangDlg::OnBnClickedROptionccd)
	ON_BN_CLICKED(IDC_R_CloseCCD, &SHHuangDlg::OnBnClickedRCloseccd)
	ON_BN_CLICKED(IDC_Start, &SHHuangDlg::OnBnClickedStart)
	ON_BN_CLICKED(IDC_Pause, &SHHuangDlg::OnBnClickedPause)
	ON_BN_CLICKED(IDC_Stop, &SHHuangDlg::OnBnClickedStop)
	ON_BN_CLICKED(IDC_Continue, &SHHuangDlg::OnBnClickedContinue)
	ON_BN_CLICKED(IDC_BUTTON1, &SHHuangDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON4, &SHHuangDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON_ReadFeature, &SHHuangDlg::OnBnClickedButtonReadfeature)
	ON_BN_CLICKED(IDC_BUTTON_big, &SHHuangDlg::OnBnClickedButtonbig)
	ON_BN_CLICKED(IDC_BUTTON_small, &SHHuangDlg::OnBnClickedButtonsmall)
	ON_BN_CLICKED(IDC_BUTTON_UP, &SHHuangDlg::OnBnClickedButtonUp)
	ON_BN_CLICKED(IDC_BUTTON_DOWN, &SHHuangDlg::OnBnClickedButtonDown)
	ON_BN_CLICKED(IDC_BUTTON_LEFT, &SHHuangDlg::OnBnClickedButtonLeft)
	ON_BN_CLICKED(IDC_BUTTON_RIGHT, &SHHuangDlg::OnBnClickedButtonRight)
	ON_BN_CLICKED(IDC_BUTTON_Reset, &SHHuangDlg::OnBnClickedButtonReset)
	ON_BN_CLICKED(IDC_SOCKET_CONNECT, &SHHuangDlg::OnBnClickedSocketConnect)
	ON_BN_CLICKED(IDC_BUTTON_Connect_I90, &SHHuangDlg::OnBnClickedButtonConnectI90)
	ON_WM_KEYDOWN()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEMOVE()
END_MESSAGE_MAP()


// CYJHuangDlg 訊息處理常式

BOOL SHHuangDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

	// TODO: 在此加入額外的初始設定

	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);


	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	//20160131
	m_Update.SetCheck(1);
	m_Update.EnableWindow(1);


	//影像貼圖視窗位置及大小設定
	m_L_Image_Live_c.SetWindowPos(&wndTop, 13, 90, 320, 240, SWP_SHOWWINDOW);
	m_L_Image_SURF_c.SetWindowPos(&wndTop, 13, 350, 320, 240, SWP_SHOWWINDOW);
	m_R_Image_Live_c.SetWindowPos(&wndTop, 350, 90, 320, 240, SWP_SHOWWINDOW);
	m_R_Image_SURF_c.SetWindowPos(&wndTop, 350, 350, 320, 240, SWP_SHOWWINDOW);
	//	m_Image_PhotoCount_c.SetWindowPos(&wndTop, 687, 190, 32, 16, SWP_SHOWWINDOW);
	m_Map_c.SetWindowPos(&wndTop, 687, 215, 600, 500, SWP_SHOWWINDOW);



	pWnd_map = GetDlgItem(IDC_Map);
	pWnd_map->GetWindowRect(rect_map);

	//*****************************************L_CCD選擇*******************************************************
	char L_sName[100];
	char L_CameraName[100];

	for (int i = 0; i < L_Cam.GetCaptureCount(); i++)
	{
		L_Cam.GetDeviceName(i, L_sName, 100);
		sprintf_s(L_CameraName, "%d %s", i, L_sName);
		m_L_SelectCamera_c.AddString((CString)L_CameraName);
	}

	m_L_SelectCamera_c.SetCurSel(0);
	//******************************************************************************************************



	//*****************************************R_CCD選擇*******************************************************
	char R_sName[100];
	char R_CameraName[100];

	for (int i = 0; i < R_Cam.GetCaptureCount(); i++)
	{
		R_Cam.GetDeviceName(i, R_sName, 100);
		sprintf_s(R_CameraName, "%d %s", i, R_sName);
		m_R_SelectCamera_c.AddString((CString)R_CameraName);
	}

	m_R_SelectCamera_c.SetCurSel(0);
	//******************************************************************************************************


	L_InitialCCD = false;
	R_InitialCCD = false;

	PhotoCount = 0;

	Continue_car_draw = TRUE;
	Thread_Info_car_draw.hWnd = m_hWnd;
	Thread_Info_car_draw.Continue = &Continue_car_draw;
	m_pThread_car_draw = AfxBeginThread(ThreadFun_car_draw, (LPVOID)&Thread_Info_car_draw);

	AfxSocketInit();
	m_socket.registerParent(this);
	m_socket_ip_c.SetAddress(192, 168, 1, 210);
	m_socket_port = 25650;

	UpdateData(false);

	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
}

void SHHuangDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void SHHuangDlg::OnPaint() //地圖大小在txt檔
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}

}


// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR SHHuangDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void SHHuangDlg::OnBnClickedLInitialccd()
{
	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(1);
	m_L_CloseCCD_c.EnableWindow(1);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	L_Cam.CcdInitial(m_L_Image_Live_c.m_hWnd, m_L_SelectCamera_c.GetCurSel());
	L_Cam.Capture();
	L_InitialCCD = true;

	if (L_InitialCCD && R_InitialCCD)
	{
		m_LoadImage_Mode_c.SetCheck(0);
		m_OnLine_Mode_c.SetCheck(1);

		m_LoadImage_Mode_c.EnableWindow(1);
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}
}


void SHHuangDlg::OnBnClickedLOptionccd()
{
	L_Cam.ConfigVideoFilter();
}


void SHHuangDlg::OnBnClickedLCloseccd()
{
	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);

	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	L_Cam.StopCapture();
	L_InitialCCD = false;
}


void SHHuangDlg::OnBnClickedRInitialccd()
{
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(1);
	m_R_CloseCCD_c.EnableWindow(1);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	R_Cam.CcdInitial(m_R_Image_Live_c.m_hWnd, m_R_SelectCamera_c.GetCurSel());
	R_Cam.Capture();
	R_InitialCCD = true;

	if (L_InitialCCD && R_InitialCCD)
	{
		m_LoadImage_Mode_c.SetCheck(0);
		m_LoadImage_Mode_Laser_c.SetCheck(0);
		m_OnLine_Mode_c.SetCheck(1);

		m_LoadImage_Mode_Laser_c.EnableWindow(1);
		m_LoadImage_Mode_c.EnableWindow(1);
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}
}

void SHHuangDlg::OnBnClickedROptionccd()
{
	R_Cam.ConfigVideoFilter();
}

void SHHuangDlg::OnBnClickedRCloseccd()
{
	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);

	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	R_Cam.StopCapture();
	L_InitialCCD = false;
}

void CALLBACK TimeProc(UINT uTimerID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	SHHuangDlg* pointer = (SHHuangDlg*)dwUser;
	pointer->DoEvent(); //要重複執行的函式
}

UINT SHHuangDlg::ThreadFun_car_draw(LPVOID lParam)
{
	THREAD_INFO_car_draw* Thread_Info = (THREAD_INFO_car_draw *)lParam;
	SHHuangDlg * hWnd = (SHHuangDlg *)CWnd::FromHandle((HWND)Thread_Info->hWnd);

	CWnd* pWnd_IDC_Map = (CWnd *)hWnd->GetDlgItem(IDC_Map);
	CRect Rect_Map;
	pWnd_IDC_Map->GetWindowRect(Rect_Map);

	CDC* dc = pWnd_IDC_Map->GetWindowDC();


	orgin.x = (Rect_Map.right - Rect_Map.left) / 2;
	orgin.y = (Rect_Map.bottom - Rect_Map.top) / 2;
	int i = 0;
	int match_size = 0;
	int k = 0;
	int j = 0;
	int P1X, P1Y;
	int P2X, P2Y;
	char xx[100];
	float hessian_min = 1000000;
	CvFont Font1 = cvFont(1, 1);
	CvFont Font2 = cvFont(0.9, 1);
	IplImage * draw_data = NULL;
	draw_data = cvCreateImage(cvSize(Rect_Map.right - Rect_Map.left, Rect_Map.bottom - Rect_Map.top), IPL_DEPTH_8U, 3);
	CvPoint draw_car[3];
	vector <vector<Keep_Feature>> feature_path_temp;
	while (1)
	{
		cvLine(draw_data, cvPoint(orgin.x + map_move[0], -100 * scale +map_move[1]), cvPoint(orgin.x + map_move[0], Rect_Map.bottom + 100 * scale + map_move[1]), CV_RGB(125, 125, 125), 1);
		cvLine(draw_data, cvPoint(-100 * scale + map_move[0], orgin.y + map_move[1]), cvPoint(Rect_Map.right + 100 * scale + map_move[0], orgin.y + map_move[1]), CV_RGB(125, 125, 125), 1);

		if (Path.size() > 1)
		{

			for (i = 1; i < Path.size(); i++)  //畫路徑
			{
				int x1 = cvRound(orgin.x + Path[i - 1].x  * scale + map_move[0] + map_move[2]);
				int y1 = cvRound(orgin.y - Path[i - 1].y * scale + map_move[1] + map_move[3]);
				int x2 = cvRound(orgin.x + Path[i].x  * scale + map_move[0] + map_move[2]);
				int y2 = cvRound(orgin.y - Path[i].y  * scale + map_move[1] + map_move[3]);
				cvLine(draw_data, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(0, 255, 0), 1);
			}

			draw_car[0].x = cvRound(orgin.x + Path[i - 1].x  * scale + 9 * sin(-Camera[5])*scale / initial_scale) + map_move[0] + map_move[2];
			draw_car[0].y = cvRound(orgin.y - Path[i - 1].y  * scale - 9 * cos(-Camera[5])*scale / initial_scale) + map_move[1] + map_move[3];
			draw_car[1].x = cvRound(orgin.x + Path[i - 1].x  * scale - 5 * cos(-Camera[5])*scale / initial_scale - 5 * sin(-Camera[5])*scale / initial_scale) + map_move[0] + map_move[2];
			draw_car[1].y = cvRound(orgin.y - Path[i - 1].y  * scale - 5 * sin(-Camera[5])*scale / initial_scale + 5 * cos(-Camera[5])*scale / initial_scale) + map_move[1] + map_move[3];
			draw_car[2].x = cvRound(orgin.x + Path[i - 1].x  * scale + 5 * cos(-Camera[5])*scale / initial_scale - 5 * sin(-Camera[5])*scale / initial_scale) + map_move[0] + map_move[2];
			draw_car[2].y = cvRound(orgin.y - Path[i - 1].y  * scale + 5 * sin(-Camera[5])*scale / initial_scale + 5 * cos(-Camera[5])*scale / initial_scale) + map_move[1] + map_move[3];
//			cvFillConvexPoly(draw_data, draw_car, 3, CV_RGB(255, 255, 0), CV_AA, 0);  //畫三角形車車

			if (feature_path.size() > 30)
			{
				feature_path_temp.assign(feature_path.end() - 20, feature_path.end());
				feature_path.clear();
				feature_path.reserve(30);
				feature_path.assign(feature_path_temp.begin(), feature_path_temp.end());
				feature_path_temp.clear();
			}

			for (k = feature_path.size() - 15; k < feature_path.size(); k++)  //畫特徵點軌跡
			{
				for (j = 0; j < feature_path[k].size(); j++)
				{
					P1X = cvRound(orgin.x + feature_path[k][j].X * scale + map_move[0] + map_move[2]);
					P1Y = cvRound(orgin.y - feature_path[k][j].Y * scale + map_move[1] + map_move[3]);

					if (k == feature_path.size() - 1)
					{
						float hessian_temp = feature_path[feature_path.size() - 1][j].hessian;
						if (hessian_min > hessian_temp)
							hessian_min = hessian_temp;

						if (!feature_path[feature_path.size() - 1][j].match)
							cvCircle(draw_data, cvPoint(P1X, P1Y), 1, CV_RGB(0, 255, 255), CV_FILLED, CV_FILLED, 0);
						else
						{
							cvCircle(draw_data, cvPoint(P1X, P1Y), 2, CV_RGB(255, 0, 0), CV_FILLED, CV_FILLED, 0);
							match_size++;
							sprintf_s(xx, "%d", feature_path[feature_path.size() - 1][j].num);
							cvPutText(draw_data, xx, cvPoint(P1X + 5, P1Y - 5), &Font2, CV_RGB(200, 255, 255));
						}
					}


					if (!feature_path[k][j].appear || feature_path[k][j].X == 0 || feature_path[k - 1][j].X == 0 || feature_path[k - 1][j].Y == 0 || feature_path[k][j].Y == 0 || !feature_path[k][j].match)
						continue;


					if (feature_path[k][j].num == feature_path[k - 1][j].num)
					{
						P2X = cvRound(orgin.x + feature_path[k - 1][j].X * scale + map_move[0] + map_move[2]);
						P2Y = cvRound(orgin.y - feature_path[k - 1][j].Y * scale + map_move[1] + map_move[3]);
						cvLine(draw_data, cvPoint(P2X, P2Y), cvPoint(P1X, P1Y), CV_RGB(150, 150, 255), 1);
					}
				}

			}

		}
		else
		{
			draw_car[0].x = cvRound(orgin.x) + map_move[0];
			draw_car[0].y = cvRound(orgin.y - 9) + map_move[1];
			draw_car[1].x = cvRound(orgin.x - 5) + map_move[0];
			draw_car[1].y = cvRound(orgin.y + 5) + map_move[1];
			draw_car[2].x = cvRound(orgin.x + 5) + map_move[0];
			draw_car[2].y = cvRound(orgin.y + 5) + map_move[1];
			cvFillConvexPoly(draw_data, draw_car, 3, CV_RGB(255, 255, 0), CV_AA, 0);
		}

		sprintf_s(xx, "(%d, %d, %0.2f, %0.5f)", draw_feature.size(), match_size, hessian_min, 98.2 / scale);
		cvPutText(draw_data, xx, cvPoint(10, 20), &Font1, CV_RGB(255, 255, 255));

		char yy[30] = "Version V2";
		cvPutText(draw_data, yy, cvPoint(5, Rect_Map.bottom - 250), &Font2, CV_RGB(100, 100, 100));

		match_size = 0;

		CvvImage show1;
		show1.CopyOf(draw_data);
		if (check_updatedata)
			show1.Show(*dc, 0, 0, draw_data->width, draw_data->height);

		memset((unsigned char*)draw_data->imageData, 0, draw_data->imageSize);

		Sleep(30 - Path.size() / 300);
	}
	*Thread_Info->Continue = false;
	::PostMessage(hWnd->m_hWnd, WNU_THREAD_EXIT, 0, 0);
	return(0);
}

UINT SHHuangDlg::ThreadFun_TARGET_control(LPVOID lParam)
{
	bool firstin = 1;
	THREAD_INFO_TARGET_control* Thread_Info = (THREAD_INFO_TARGET_control *)lParam;
	SHHuangDlg * hWnd = (SHHuangDlg *)CWnd::FromHandle((HWND)Thread_Info->hWnd);

	CString str_Value, str_Value2, str_Value3, str_Value4, str_Value5, str_Value6;
	CStatic * Static_num = (CStatic *)hWnd->GetDlgItem(IDC_I90_VR);
	CStatic * Static_num2 = (CStatic *)hWnd->GetDlgItem(IDC_I90_VL);

	target_pos[0] = 0;
	target_pos[1] = 0;
	target_pos[2] = 0;  //旋轉角
	
	//	int state = 1;
	int rho_gain;
	int times = 0;
	float M1, M2, N1, N2;
	float alpha_gain;
	float beta_gain;
	double rho = 0;
	double theta = 0;
	double alpha = 0;
	double beta = 0;
	double phi = 0;
	sampleTime = 50; //毫秒

	remove("control_output.txt");
	remove("control_output_m.txt");
	remove("control_pos_m.txt");
	fstream app_control("control_output.txt", ios::app);
	fstream app_control_m("control_output_m.txt", ios::app); //為了給matlab畫圖用的
	fstream app_pos_m("control_pos_m.txt", ios::app);

	while (carFLAG)
	{
		car_x = Camera[0] - 0 /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
		car_y = Camera[1] - 0 /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
		car_zdir = CRSocket::Laser_pos[2]  /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;


		phi = car_zdir;
		if (phi > PI)		phi = -2 * PI + phi;
		if (phi < -PI)		phi = 2 * PI + phi;

		rho = sqrt((car_x - target_pos[0])*(car_x - target_pos[0]) + (car_y - target_pos[1])*(car_y - target_pos[1]));
		theta = atan2(car_y - target_pos[1], car_x - target_pos[0]);


		alpha = -phi + theta + PI;
		if (alpha > PI)		alpha = -2 * PI + alpha;
		if (alpha < -PI)		alpha = 2 * PI + alpha;

		beta = -(theta + PI) + target_pos[2];
		if (beta > PI)		beta = -2 * PI + beta;
		if (beta < -PI)		beta = 2 * PI + beta;

		//----------------------第二模式新增之判斷------------------------------------------------------------
		float P2_1 = 1.1737;
		float P2_2 = 1.4317;
		float P2_3 = 0.2422;
		float P2_4 = 0.4223;

		float Vt2 = rho * rho * P2_1 +
			alpha*alpha*P2_2 +
			alpha*phi*P2_4 +
			alpha*phi*P2_4 +
			phi*phi*P2_3;

		//-----------------------------------------------------------------------------------------------------

#if 1

		//原始線性控制
		vr = 3 * rho + 0.15 * (8 * alpha - 4 * (beta));
		vl = 3 * rho - 0.15 * (8 * alpha - 4 * (beta));
		vr = vr / 10;
		vl = vl / 10;

#else
		//判斷要切換哪種模式TS-FUZZY
		if (times == 0 || alpha > (PI / 10) || alpha < (-PI / 10))
		{
			if (state == 3)
				state = 3;

			else if (state == 2)
				state = 2;

			else
				state = 1;
		}
		else if ((rho > 0.6 || Vt2 > 7))  //gamma = 2
		{
			// 			if (state == 3)
			// 				state = 3;
			// 			else

			if (firstin)
			{
				I90_PWM_send(0, 0);

				Sleep(1000);
			}
			firstin = 0;
			state = 2;
		}
		else
			state = 3;

		if (alpha < PI / 10 && alpha>-PI / 10)
			times = 1; //一開始必定Mode1，其餘只看角度來決定是否進入Mode1

					   //切換式TS-Fuzzy控制
		alpha_gain = 1.9161;

		switch (state)
		{
		case 1:  //旋轉Mode
				 //		alpha_gain = 10 * (alpha / pi);

			vr = 0 * rho + 0.15 * (alpha_gain*alpha - 0 * (beta));
			vl = 0 * rho - 0.15 * (alpha_gain*alpha - 0 * (beta));
			vr = vr *1.4;
			vl = vl *1.4;

			break;

		case 2:  //直線Mode
				 // 			rho_gain = (4 * rho / 200) + (1 - (rho / 200));
				 // 			if (rho > 150) rho_gain = 4;
			rho_gain = 1.0331;
			vr = rho_gain *rho + 0.15 * (alpha_gain*alpha - 0 * (beta));
			vl = rho_gain *rho - 0.15 * (alpha_gain*alpha - 0 * (beta));
			vr = vr*0.1;
			vl = vl*0.1;
			break;

		case 3:  //PDC Mode
				 //			rho_gain = (3 * rho / 125) + (1 - (rho / 125));
			rho_gain = 1.0331;
			M1 = (cos(alpha) - 0.031415926) / (1 - 0.031415926);
			M2 = 1 - M1;
			//		M2 = (1 - cos(alpha)) / (1 - 0.031415926);

			if (alpha == 0)
				N1 = 1;
			else
				N1 = (0.49*PI * sin(alpha) - sin(0.49*PI)*alpha) / (alpha*(0.49*PI - sin(0.49*PI)));

			N2 = 1 - N1;

			// 		alpha_gain = M1*N1*5.8766 +
			// 			                  M1*N2*5.6385 +
			// 			                  M2*N1*5.8766 +
			// 			                  M2*N2*5.6385;
			// 
			// 		beta_gain = M1*N1*1.1052 +
			// 			                M1*N2*1.0776 +
			// 			                M2*N1*1.1052 +
			// 			                M2*N2*1.0776;

			alpha_gain = M1*N1*1.2833 +
				M1*N2*1.1022 +
				M2*N1* 1.2833 +
				M2*N2*1.1022;

			beta_gain = -M1*N1*0.0487 +
				-M1*N2*0.0517 +
				-M2*N1*0.0487 +
				-M2*N2*0.0517;

			// 			beta_gain = -M1*N1*1.2833 +
			// 				-M1*N2*1.1022 +
			// 				-M2*N1* 1.2833 +
			// 				-M2*N2*1.1022;
			// 
			// 			alpha_gain = M1*N1*0.0487 +
			// 				M1*N2*0.0517 +
			// 				M2*N1*0.0487 +
			// 				M2*N2*0.0517;


			vr = rho_gain * rho + 0.15 * (alpha_gain * alpha - beta_gain * (beta));
			vl = rho_gain * rho - 0.15 * (alpha_gain * alpha - beta_gain * (beta));
			vr = vr*0.3;
			vl = vl*0.3;
			break;

		default:
			break;
		}
#endif

		// 		if (vr > 10)
		// 			vr = 10;
		// 
		// 		if (vl > 10)
		// 			vl = 10;
		// 
		// 
		// 		if (vr < -10)
		// 			vr = -10;
		// 
		// 		if (vl < -10)
		// 			vl = -10;

		vr_draw = vr;
		vl_draw = vl;


		vr = vr * 3400;
		vl = vl * 3400;

		if (vr > 0)
			vr = vr + 8100;
		else
			vr = vr - 8100;

		if (vl > 0)
			vl = vl + 8100;
		else
			vl = vl - 8100;

		app_control_m
			<< setw(12) << setprecision(5) << rho
			<< setw(12) << setprecision(5) << theta
			<< setw(12) << setprecision(5) << phi
			<< setw(12) << setprecision(5) << alpha
			<< setw(12) << setprecision(5) << beta
			<< setw(12) << setprecision(5) << vl
			<< setw(12) << setprecision(5) << vr
			<< setw(12) << setprecision(5) << state
			<< endl;

		app_pos_m
			<< car_zdir << "  "
			<< car_x << "  "
			<< car_y << "  "
			<< endl;



		// 		if (state == 3)
		// 		{
		// 			if (vr > 0)
		// 				vr = vr -150;
		// 			else
		// 				vr = vr + 150;
		// 
		// 			if (vl > 0)
		// 				vl = vl - 150;
		// 			else
		// 				vl = vl + 150;
		// 		}

		I90_PWM_send(vl, vr);

		cout.flags(ios::left);

		str_Value.Format(_T("%0.2f"), vr);
		Static_num->SetWindowText(str_Value);

		str_Value2.Format(_T("%0.2f"), vl);
		Static_num2->SetWindowText(str_Value2);

		// 		app_control
		// 			<< " |rho= " << setw(7) << setprecision(4) << rho << setw(6)
		// 			<< " |theta= " << setw(7) << setprecision(4) << theta << setw(6)
		// 			<< " |phi= " << setw(7) << setprecision(4) << phi << setw(6)
		// 			<< " |alpha= " << setw(10) << setprecision(4) << alpha << setw(6)
		// 			<< " |beta= " << setw(10) << setprecision(4) << beta << setw(4)
		// 			<< " |vl= " << setw(5) << setprecision(4) << vl << setw(4)
		// 			<< " |vr= " << setw(5) << setprecision(4) << vr
		// 			<< endl;


		Sleep(sampleTime);
	}




	*Thread_Info->Continue = false;
	::PostMessage(hWnd->m_hWnd, WNU_THREAD_EXIT, 0, 0);
	return(0);
}

void SHHuangDlg::DoEvent()
{
	time1 = (double)cvGetTickCount(); //***********************************************************************************

	//fstream app_Test("Test.txt", ios::app);
	//app_Test << PhotoCount << endl;

	if (m_LoadImage_Mode_c.GetCheck())
	{
//		PhotoCount = 500;
		SampleTime = SampleTime_temp[PhotoCount]; // 相片間隔時間

		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);



		if (in_image0 && in_image1)
		{
			time3 = (double)cvGetTickCount(); //***********************************************************************************
			IplImage* l_image = cvLoadImage(path0, CV_LOAD_IMAGE_COLOR);
			IplImage* r_image = cvLoadImage(path1, CV_LOAD_IMAGE_COLOR);
			time4 = (double)cvGetTickCount(); //***********************************************************************************
			//app_Test << "LoadImage_ok" << endl;
			WORK(l_image, r_image, SampleTime);
			//app_Test << "WORK_ok" << endl;			
			cvReleaseImage(&l_image);
			cvReleaseImage(&r_image);

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			OnBnClickedStop();

			MessageBox(TEXT("圖片讀取結束!"));
		}

		in_image0.close();
		in_image1.close();


	}
	else if (m_OnLine_Mode_c.GetCheck())
	{


		time9 = (double)cvGetTickCount(); //***********************************************************************************

		IplImage* l_image = L_Cam.GetOneImage();
		IplImage* r_image = R_Cam.GetOneImage();

		time5 = (double)cvGetTickCount(); //***********************************************************************************
		//app_Test << "GetImage_ok" << endl;
		if (m_SaveImage_Mode_c.GetCheck())
		{
			_mkdir("photo"); // 建立資料夾

			char path0[100];
			sprintf_s(path0, "photo\\L%d.png", PhotoCount);
			cvSaveImage(path0, l_image);

			char path1[100];
			sprintf_s(path1, "photo\\R%d.png", PhotoCount);
			cvSaveImage(path1, r_image);

			if (BinocularSLAM.Online_Laser_Localization_enable)
			{
				fstream app_Laser_Data("Laser_Data.txt", ios::app);
				app_Laser_Data << CRSocket::Laser_pos[0] << " " << CRSocket::Laser_pos[1] << " " << CRSocket::Laser_pos[2] << endl;
				app_Laser_Data.close();
			}
			fstream app_SampleTime("photo\\SampleTime.txt", ios::app);
			app_SampleTime << SampleTime << endl;

			app_SampleTime.close();

			//app_Test << "SaveImage_ok" << endl;
		}

		time6 = (double)cvGetTickCount(); //***********************************************************************************


		WORK(l_image, r_image, SampleTime);
		//app_Test << "WORK_ok" << endl;
		cvReleaseImage(&l_image);
		cvReleaseImage(&r_image);

		CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
		ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);


		PhotoCount++;

	}
	else if (m_LoadImage_Mode_Laser_c.GetCheck())
	{
		SampleTime = SampleTime_temp[PhotoCount]; // 相片間隔時間

		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);

		BinocularSLAM.Offline_Laser_Localization_enable = true;

		if (in_image0 && in_image1)
		{
			time3 = (double)cvGetTickCount(); //***********************************************************************************
			IplImage* l_image = cvLoadImage(path0, CV_LOAD_IMAGE_COLOR);
			IplImage* r_image = cvLoadImage(path1, CV_LOAD_IMAGE_COLOR);
			time4 = (double)cvGetTickCount(); //***********************************************************************************
											  //app_Test << "LoadImage_ok" << endl;
			WORK(l_image, r_image, SampleTime);
			//app_Test << "WORK_ok" << endl;			
			cvReleaseImage(&l_image);
			cvReleaseImage(&r_image);

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			OnBnClickedStop();

			MessageBox(TEXT("圖片讀取結束!"));
		}

		in_image0.close();
		in_image1.close();
	}









	time2 = (double)cvGetTickCount(); //***********************************************************************************


	m_Time1 = (int)((time2 - time1) / (cvGetTickFrequency()*1000.)); // 系統全部
	m_Frequency2 = cvRound(1. / (((time2 - time1) / (cvGetTickFrequency()*1000.)) / 1000.)); // 系統全部 頻率

	if (m_LoadImage_Mode_c.GetCheck() || m_LoadImage_Mode_Laser_c.GetCheck())
	{
		m_Time2 = 0; // 擷圖
		m_Time3 = (int)((time4 - time3) / (cvGetTickFrequency()*1000.)); // 讀圖
	}
	else if (m_OnLine_Mode_c.GetCheck())
	{
		m_Time2 = (int)((time5 - time9) / (cvGetTickFrequency()*1000.)); // 擷圖
		m_Time3 = 0; // 存讀圖 
		SampleTime = ((time2 - time1) / (cvGetTickFrequency()*1000.)) / 1000.; // 相片間隔時間

		if (m_SaveImage_Mode_c.GetCheck())
		{
			m_Time3 = (int)((time6 - time5) / (cvGetTickFrequency()*1000.)); // 存圖

			fstream app_GetImageTime("photo\\GetImageTime.txt", ios::app);
			app_GetImageTime << ((time5 - time9) / (cvGetTickFrequency()*1000.)) / 1000. << endl;
			app_GetImageTime.close();


			fstream app_SaveImageTime("photo\\SaveImageTime.txt", ios::app);
			app_SaveImageTime << ((time6 - time5) / (cvGetTickFrequency()*1000.)) / 1000. << endl;
			app_SaveImageTime.close();
		}
	}



	m_SampleTime = SampleTime; // 相片間隔時間
	if (SampleTime > 0)   m_Frequency = cvRound(1. / SampleTime); // 相片間隔 頻率

	m_cameraX = BinocularSLAM.p3dx_x.x * 100;
	m_cameraY = BinocularSLAM.p3dx_x.y * 100;
	m_cameraZ = BinocularSLAM.p3dx_x.z * 100;
	m_cameraSitaX = BinocularSLAM.p3dx_x.x_dir * 180 / CV_PI;
	m_cameraSitaY = BinocularSLAM.p3dx_x.y_dir * 180 / CV_PI;
	m_cameraSitaZ = BinocularSLAM.p3dx_x.z_dir * 180 / CV_PI;

	check_updatedata = 0;
	UpdateData(false);
	check_updatedata = 1;

}



void SHHuangDlg::ShowImage(const IplImage* image, CWnd* pWnd)
{

	IplImage* image_show = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);

	if (image->width == 320 && image->height == 240)
	{
		cvCopy(image, image_show); //圖片複製
	}
	else
	{
		cvResize(image, image_show, CV_INTER_LINEAR); //圖片縮放
	}


	CDC* dc = pWnd->GetWindowDC();
	CvvImage show;
	//CImage show;	
	show.CopyOf(image_show);
	show.Show(*dc, 0, 0, image_show->width, image_show->height);
	cvReleaseImage(&image_show);
	ReleaseDC(dc);

}


//****************************************************************************************************************************************SLAM

void SHHuangDlg::WORK(const IplImage* l_image, const IplImage* r_image, const double SampleTime)
{
	//	fstream app_p3dx("isruning.txt", ios::app);

	BinocularSLAM.Run_BinocularSLAM(l_image, r_image, SampleTime);

	Sleep(sleep_time);
	//	InvalidateRect(rect_map); // 小地圖更新  (20160620拿掉，不需要了)
	save_car_pos();  //20160620 改版繪圖

	IplImage* l_image_color = cvCreateImage(cvGetSize(l_image), IPL_DEPTH_8U, 3);
	cvCopy(l_image, l_image_color);

	IplImage* r_image_color = cvCreateImage(cvGetSize(r_image), IPL_DEPTH_8U, 3);
	cvCopy(r_image, r_image_color);

	//----------------------------------------------------------------------------------------------------keyfeature
	for (int i = 0; i < (int)BinocularSLAM.map_feature.size(); i++) // 畫出顯示影像上地圖特徵點
	{
		if (!BinocularSLAM.map_feature[i].match) continue;

		int l_ix = cvRound(BinocularSLAM.map_feature[i].l_ix);
		int l_iy = cvRound(BinocularSLAM.map_feature[i].l_iy);
		int r_ix = cvRound(BinocularSLAM.map_feature[i].r_ix);
		int r_iy = cvRound(BinocularSLAM.map_feature[i].r_iy);
		int r = cvRound(BinocularSLAM.map_feature[i].size*1.2 / 9. * 2);

		if (BinocularSLAM.map_feature[i].theMAP)
		{
			cvRectangle(l_image_color, cvPoint(l_ix - r, l_iy + r), cvPoint(l_ix + r, l_iy - r), CV_RGB(255, 200, 0), 2, CV_AA, 0);
			cvRectangle(r_image_color, cvPoint(r_ix - r, r_iy + r), cvPoint(r_ix + r, r_iy - r), CV_RGB(255, 200, 0), 2, CV_AA, 0);
		}
		else
		{
			cvRectangle(l_image_color, cvPoint(l_ix - r, l_iy + r), cvPoint(l_ix + r, l_iy - r), CV_RGB(0, 0, 255), 2, CV_AA, 0);
			cvRectangle(r_image_color, cvPoint(r_ix - r, r_iy + r), cvPoint(r_ix + r, r_iy - r), CV_RGB(0, 0, 255), 2, CV_AA, 0);
		}
		cvLine(l_image_color, cvPoint(0, 120), cvPoint(320, 120), CV_RGB(0, 255, 125), 1);
		cvLine(l_image_color, cvPoint(160, 0), cvPoint(160, 240), CV_RGB(0, 255, 125), 1);
	}
	//----------------------------------------------------------------------------------------------------
	CWnd* pWnd_L_Image_SURF = GetDlgItem(IDC_L_Image_SURF);
	CWnd* pWnd_R_Image_SURF = GetDlgItem(IDC_R_Image_SURF);
	ShowImage(l_image_color, pWnd_L_Image_SURF);
	ShowImage(r_image_color, pWnd_R_Image_SURF);
	//////////////
	_mkdir("photo2"); // 建立資料夾

	char path0[100];
	sprintf_s(path0, "photo2\\L%d.jpg", PhotoCount);

	CDC* dc = pWnd_L_Image_SURF->GetWindowDC();
	CDC* dc1 = pWnd_R_Image_SURF->GetWindowDC();

	char num[10];

	CvScalar Color;
	Color = CV_RGB(255, 0, 0);
	CvFont Font1 = cvFont(1.3, 2);

	draw_feature.assign(BinocularSLAM.map_feature.begin(), BinocularSLAM.map_feature.end()); // draw_feature 複制 BinocularSLAM.map_feature
	feature_path.push_back(draw_feature);  //繪製特徵點軌跡

	if (m_FollowMode_c.GetCheck())  //追隨模式
	{
		// 		orgin.x = Path[Path.size()].x  * scale + map_move[0];
		// 		orgin.y = Path[Path.size()].y  * scale + map_move[1];



// 		map_move[2] = Path[Path.size() - 1].x  * scale;
// 		map_move[3] = Path[Path.size() - 1].y  * scale;
// 
// 		orgin.x = 300 + map_move[2];
// 		orgin.y = 250 + map_move[3];

	}

	for (int i = 0; i < (int)BinocularSLAM.map_feature.size(); i++) // 顯示影像上地圖特徵點編號
	{
		if (!BinocularSLAM.map_feature[i].match) continue;

		int l_ix = cvRound(BinocularSLAM.map_feature[i].l_ix);
		int l_iy = cvRound(BinocularSLAM.map_feature[i].l_iy);
		int r_ix = cvRound(BinocularSLAM.map_feature[i].r_ix);
		int r_iy = cvRound(BinocularSLAM.map_feature[i].r_iy);
		int r = cvRound(BinocularSLAM.map_feature[i].size*1.2 / 9. * 2);


		CvPoint TextPosition1 = cvPoint(BinocularSLAM.map_feature[i].l_ix - 10, BinocularSLAM.map_feature[i].l_iy - 10);

		sprintf_s(num, "%d", BinocularSLAM.map_feature[i].num);
		cvPutText(l_image_color, num, TextPosition1, &Font1, Color);

		dc->TextOut(l_ix + 8, l_iy - 8, (CString)num);
		dc1->TextOut(r_ix + 8, r_iy - 8, (CString)num);
	}

	cvSaveImage(path0, l_image_color);

	/////////////
	cvReleaseImage(&l_image_color);
	cvReleaseImage(&r_image_color);

	CWnd* pWnd_Map = GetDlgItem(IDC_Map);
	CDC* dc2 = pWnd_Map->GetWindowDC();

	dc2->SetBkColor(RGB(0, 0, 0));
	dc2->SetTextColor(RGB(255, 255, 255));

	char finish[10] = "Finish";

	if (move_turn == 3)
	{
		dc2->TextOut(0, 0, (CString)finish);
	}


	////



	ReleaseDC(dc);
	ReleaseDC(dc1);
	ReleaseDC(dc2);


	if (BinocularSLAM.map_feature.size())
	{

		//		fstream app_match("match.txt", ios::app);
		fstream app_world_feature("world_feature.txt", ios::app);
		//		fstream app_print("print.txt", ios::app);
				//fstream app_print_feature_descriptor("FeatureDescriptor12345.txt", ios::app);
		if (PhotoCount > 0)
		{
			int on_image_num1 = 0;
			for (int i = 0; i < BinocularSLAM.map_feature.size(); i++)
			{
				if (BinocularSLAM.map_feature[i].match)
				{
					on_image_num1++;
				}
			}

			//			app_match << setw(8) << PhotoCount << "  "  << setw(8) << BinocularSLAM.map_feature.size() << setw(8) << "  " <<on_image_num1 <<endl;
//			app_print << setw(8) << PhotoCount << "  " << setw(8) << BinocularSLAM.map_feature.size() << setw(8) << "  " << on_image_num1 << "  " << "0" << "  " << "0" << "  " << "0" << "  " << "0" << "  " << "0" << endl;

			if (PhotoCount % 40 == 0)
			{
				app_world_feature.close();
				remove("world_feature.txt");
				fstream app_world_feature("world_feature.txt", ios::app);
			}

			app_world_feature << "20" << endl;

			for (int i = 0; i < BinocularSLAM.map_feature.size(); i++)
			{

				//-------------------20160608新增儲存地標點資訊--------------------------------------

				if (!BinocularSLAM.read_map_mode)
				{
					app_world_feature << BinocularSLAM.map_feature[i].X << " " << BinocularSLAM.map_feature[i].Y << " " << BinocularSLAM.map_feature[i].Z << " ";
					app_world_feature << BinocularSLAM.map_feature[i].hessian << " ";

					for (int j = 0; j < 16; j++)
						app_world_feature << BinocularSLAM.map_feature[i].average_descriptor[j] << " ";

					app_world_feature << endl;
				}
				//----------------------------------------------------------------------------------------
				// 				app_match<< setw(8)<<BinocularSLAM.map_feature[i].num<< setw(8)<<BinocularSLAM.map_feature[i].match<<endl;
				// 				if(!BinocularSLAM.map_feature[i].match) continue;
				// 				app_print << setw(8) << BinocularSLAM.map_feature[i].num << "  " << setw(8) << BinocularSLAM.map_feature[i].hx << "  " << setw(8) << BinocularSLAM.map_feature[i].hy  << "  " << setw(8) << BinocularSLAM.map_feature[i].hz << "  " ;
				// 				app_print << setw(8) << BinocularSLAM.map_feature[i].l_ix << "  " << setw(8) << BinocularSLAM.map_feature[i].l_iy << "  " << setw(8) << BinocularSLAM.map_feature[i].r_ix << "  " << setw(8) << BinocularSLAM.map_feature[i].r_iy << endl;
				//----------------------------------------------------------------------------------------------------輸出keyfeature
								//app_print_feature_descriptor << setw(4) << BinocularSLAM.map_feature[i].num
								//	<< setw(12) << BinocularSLAM.map_feature[i].hx
								//	<< setw(12) << BinocularSLAM.map_feature[i].hy
								//	<< setw(12) << -BinocularSLAM.map_feature[i].hz
								//	<< setw(3) << BinocularSLAM.map_feature[i].laplacian;
								//		for (int n = 0; n < 16; n++)
								//		{
								//			app_print_feature_descriptor << setw(15) << BinocularSLAM.map_feature[i].original_descriptor[n];
								//		}
								//app_print_feature_descriptor << endl;
				//----------------------------------------------------------------------------------------------------
			}
			// 			app_print << endl << endl;
			// 			app_match << endl << endl;
			// 			app_print.close();
			// 			app_match.close();
		}
	}
}


void SHHuangDlg::OnBnClickedStart()
{
	//	UpdateData(true);
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);

	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(1);

	//if(m_LoadImage_Mode_c.GetCheck())
	{
		m_Pause_c.EnableWindow(1);
	}
	//else
	//{
	//	m_Pause_c.EnableWindow(0);
	//}



	PhotoCount = 0;
	SampleTime = 0;

	Path.swap(vector<CvPoint2D64f>());
	SampleTime_temp.swap(vector<double>());

	remove("print.txt");
	remove("point.txt");
	remove("FeatureDescriptor12345.txt");
	remove("Measurement.txt");
	remove("Test.txt");
	remove("app_address.txt");
	remove("app_address_2.txt");
	remove("txt\\feature_l.txt");
	remove("txt\\feature_r.txt");
	remove("txt\\feature_p.txt");
	remove("txt\\p3p_time.txt");
	remove("txt\\point.txt");
	remove("txt\\feature_descriptor.txt");
	remove("Comparison.txt");
	remove("txt\\feature_state.txt");
	remove("match.txt");
	remove("isruning.txt");
	remove("position.txt");
	remove("last_map_feature.txt");
	remove("root.txt");
	remove("p3dxpos.txt");
	remove("move.txt");
	remove("RRANSACtime.txt");
	remove("SURFtime.txt");
	remove("P3Ptime.txt");
	remove("TEST.txt");
	remove("world_feature.txt");

	if (BinocularSLAM.Online_Laser_Localization_enable)
	{
		remove("Laser_Data.txt");
	}

	// 	if (m_read_world_data)
	// 	{
	// 
	// 
	// 
	// 	}


	if (m_LoadImage_Mode_c.GetCheck() || m_LoadImage_Mode_Laser_c.GetCheck())
	{
		DeletePhoto2();
		fstream in_SampleTime("photo\\SampleTime.txt", ios::in);
		if (!in_SampleTime)	exit(1);

		while (!in_SampleTime.eof())
		{
			in_SampleTime >> SampleTime;
			SampleTime_temp.push_back(SampleTime);
		}

		in_SampleTime.close();
	}
	else if (m_OnLine_Mode_c.GetCheck() && m_SaveImage_Mode_c.GetCheck())
	{
		DeletePhoto();
		DeletePhoto2();
		remove("photo\\SampleTime.txt");
	}





	fstream in_Parameter("Parameter.txt", ios::in);
	if (!in_Parameter)	exit(1);

	char str[50];
	double value;

	while (in_Parameter >> str >> value)
	{
		if (!strcmp(str, "scale"))                    scale = value;
		else if (!strcmp(str, "show_image_num"))	        show_image_num = (int)value;
		else if (!strcmp(str, "show_map_num"))	            show_map_num = (int)value;
		else if (!strcmp(str, "show_feature_region"))	    show_feature_region = (int)value;
		else if (!strcmp(str, "show_Search_Window_Size"))	show_Search_Window_Size = (int)value;
		else if (!strcmp(str, "show_all_feature"))	        show_all_feature = (int)value;
		else if (!strcmp(str, "txt"))                      txt = (int)value;
	}

	in_Parameter.close();

	initial_scale = scale;


	BinocularSLAM.Initial_SLAM(); //*****************************************************************************************







	//多媒體計時器參數設定

	UINT uDelay = 1; // 自訂的取樣時間 單位:毫秒
	UINT uResolution = 1;
	DWORD dwUser = (DWORD)this;
	UINT fuEvent = TIME_PERIODIC; //You also choose TIME_ONESHOT;

	timeBeginPeriod(1); //最高取樣精度1ms
	FTimerID = timeSetEvent(uDelay, uResolution, TimeProc, dwUser, fuEvent);
}




void SHHuangDlg::OnBnClickedButtonReadfeature()
{
	fstream ReadFeature;
	double buffer[5000][30];
	string testttt;
	ReadFeature.open("world_feature_in.txt", ios::in);
	int size = 0;
	int i = 0;
	//	vector<Keep_Feature> Read_Feature;
	Keep_Feature temp_feature;

	ReadFeature >> size;
	for (i = 0; i < 123456; i++)
	{
		for (int j = 0; j < size; j++)
		{
			ReadFeature >> buffer[i][j];
		}

		temp_feature.X = buffer[i][0];
		temp_feature.Y = buffer[i][1];
		temp_feature.Z = buffer[i][2];
		temp_feature.hessian = buffer[i][3];

		for (int j = 0; j < 16; j++)
			temp_feature.average_descriptor[j] = buffer[i][4 + j];

		temp_feature.match = false;
		temp_feature.theMAP = true;
		temp_feature.num = i;
		temp_feature.laplacian = 1;
		BinocularSLAM.map_feature.push_back(temp_feature);

		if (ReadFeature.eof())
			break;
	}
	BinocularSLAM.feature_num = i;
	BinocularSLAM.read_map_mode = true;

	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	//	draw_feature.assign(BinocularSLAM.map_feature.begin(), BinocularSLAM.map_feature.end());


}

void SHHuangDlg::ShowPhotoCount(const int PhotoCount, CWnd* pWnd)
{
	CDC* dc = pWnd->GetWindowDC();

	char count[10];
	sprintf_s(count, "%0.4d", PhotoCount);
	dc->TextOut(0, 0, (CString)count);

	ReleaseDC(dc);
}

void SHHuangDlg::OnBnClickedPause()
{
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(1);
	m_Stop_c.EnableWindow(1);


	timeKillEvent(FTimerID);
	timeEndPeriod(1);
}

void SHHuangDlg::OnBnClickedStop()
{
	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_LoadImage_Mode_Laser_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);


	if (L_InitialCCD)
	{
		m_L_OptionCCD_c.EnableWindow(1);
		m_L_CloseCCD_c.EnableWindow(1);
	}

	if (R_InitialCCD)
	{
		m_R_OptionCCD_c.EnableWindow(1);
		m_R_CloseCCD_c.EnableWindow(1);
	}

	if (L_InitialCCD && R_InitialCCD)
	{
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}


	timeKillEvent(FTimerID);
	timeEndPeriod(1);
}

void SHHuangDlg::OnBnClickedContinue()
{
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(1);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(1);




	fstream in_Parameter("Parameter.txt", ios::in);
	if (!in_Parameter)	exit(1);

	char str[50];
	double value;

	while (in_Parameter >> str >> value)
	{
		if (!strcmp(str, "show_image_num"))	        show_image_num = (int)value;
		else if (!strcmp(str, "show_map_num"))	            show_map_num = (int)value;
		else if (!strcmp(str, "show_feature_region"))	    show_feature_region = (int)value;
		else if (!strcmp(str, "show_Search_Window_Size"))	show_Search_Window_Size = (int)value;
		else if (!strcmp(str, "show_all_feature"))	        show_all_feature = (int)value;
	}

	in_Parameter.close();




	//多媒體計時器參數設定

	UINT uDelay = 1; // 自訂的取樣時間 單位:毫秒
	UINT uResolution = 1;
	DWORD dwUser = (DWORD)this;
	UINT fuEvent = TIME_PERIODIC; //You also choose TIME_ONESHOT;

	timeBeginPeriod(1); //最高取樣精度1ms
	FTimerID = timeSetEvent(uDelay, uResolution, TimeProc, dwUser, fuEvent);
}



void SHHuangDlg::OnBnClickedButtonConnectI90()
{
	Connect_I90.Open(2, 115200);



	Continue_TARGET_control = TRUE;
	Thread_Info_TARGET_control.hWnd = m_hWnd;
	Thread_Info_TARGET_control.Continue = &Continue_TARGET_control;
	m_pThread_TARGET_control = AfxBeginThread(ThreadFun_TARGET_control, (LPVOID)&Thread_Info_TARGET_control);
}

void SHHuangDlg::OnBnClickedButton1()
{


}


void SHHuangDlg::OnBnClickedButton4()
{

}


void SHHuangDlg::OnBnClickedButtonbig()
{
	scale = scale*Magnification;
}


void SHHuangDlg::OnBnClickedButtonsmall()
{
	scale = scale / Magnification;

}


void SHHuangDlg::OnBnClickedButtonUp()
{
	map_move[1] += Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonDown()
{
	map_move[1] -= Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonLeft()
{
	map_move[0] += Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonRight()
{
	map_move[0] -= Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonReset()
{
	scale = initial_scale;
	map_move[0] = 0;
	map_move[1] = 0;
	map_move[2] = 0;
	map_move[3] = 0;
	orgin.x = (rect_map.right - rect_map.left) / 2;
	orgin.y = (rect_map.bottom - rect_map.top) / 2;

}

void SHHuangDlg::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值

	CDialog::OnKeyDown(nChar, nRepCnt, nFlags);
}

void SHHuangDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值
	mouse_pos = point;

	CDialog::OnMouseMove(nFlags, point);
}

BOOL SHHuangDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值
	if (zDelta > 0)
		scale = scale * Magnification;
	else
		scale = scale / Magnification;

	int x_pos = mouse_pos.x - rect_map.left;
	int y_pos = mouse_pos.y - rect_map.top;


	double x_pos2 = x_pos - orgin.x;
	double y_pos2 = y_pos - orgin.y;




	if (x_pos > 0 && y_pos > 0 && x_pos < rect_map.right - rect_map.left && y_pos < rect_map.bottom - rect_map.top)
	{
		orgin.x = x_pos;
		orgin.y = y_pos;

		if (zDelta > 0)
		{
			map_move[0] = (map_move[0] - x_pos2) * Magnification;
			map_move[1] = (map_move[1] - y_pos2) * Magnification;
		}
		else
		{
			map_move[0] = (map_move[0] - x_pos2) / Magnification;
			map_move[1] = (map_move[1] - y_pos2) / Magnification;
		}
	}

	return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

void SHHuangDlg::OnBnClickedSocketConnect()
{
	if (fg_connected) {
		DoSocketDisconnect();
	}
	else {
		DoSocketConnect();
	}
	m_socket_connect_c.SetCheck(fg_connected);

	BinocularSLAM.Online_Laser_Localization_enable = true;
	m_read_map_c.EnableWindow(0);
}


void SHHuangDlg::DeletePhoto()
{

	PhotoCount = 0;

	while (1)
	{
		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);

		if (in_image0 && in_image1)
		{
			in_image0.close();
			in_image1.close();

			remove(path0);
			remove(path1);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			in_image0.close();
			in_image1.close();

			break;
		}
	}

	PhotoCount = 0;

}

void SHHuangDlg::DeletePhoto2()
{

	PhotoCount = 0;

	while (1)
	{
		char path0[100], path1[100];
		sprintf_s(path0, "photo2\\L%d.png", PhotoCount);
		sprintf_s(path1, "photo2\\R%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		if (in_image0)
		{
			in_image0.close();

			remove(path0);
			remove(path1);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			in_image0.close();

			break;
		}
	}

	PhotoCount = 0;

}

BOOL SHHuangDlg::PreTranslateMessage(MSG * pMsg)
{
	// TODO:  在此加入特定的程式碼和 (或) 呼叫基底類別
	int upupup = 0;
	int Lpwm1 = 9500 + upupup, Rpwm1 = 9500 + upupup;
	int Lpwm2 = -9500 - upupup, Rpwm2 = -9500 - upupup;
	int Lpwm3 = -9500 - upupup, Rpwm3 = 9500 + upupup;
	int Lpwm4 = 9500 + upupup, Rpwm4 = -9500 - upupup;

	if (pMsg->message == WM_KEYDOWN)
	{
		switch (pMsg->wParam)
		{
		case 0x57:
			I90_PWM_send(Lpwm1, Rpwm1);  break;  //I90直走 w
		case 0x53:
			I90_PWM_send(Lpwm2, Rpwm2); break;  //I90後退 s
		case 0x41:
			I90_PWM_send(Lpwm3, Rpwm3); break;  //I90左轉 a
		case 0x44:
			I90_PWM_send(Lpwm4, Rpwm4); break;  //I90右轉 d
		case 0x54:
			OnBnClickedButtonUp();  break;   //小黑窗上移 t
		case 0x47:
			OnBnClickedButtonDown(); break;   //小黑窗下移 g
		case 0x46:
			OnBnClickedButtonLeft(); break;   //小黑窗左移 f
		case 0x48:
			OnBnClickedButtonRight(); break;   //小黑窗右移 h
		case 0x52:
			OnBnClickedButtonbig(); break;   //小黑窗放大 r
		case 0x59:
			OnBnClickedButtonDown(); break;   //小黑窗縮小 y
		case 0x58:
			sleep_time += 20; break;   //運算速度下降 x
		case 0x5A:
			sleep_time = 0; break;   //運算速度恢復 z
		case VK_F2:
			I90_PWM_send(0, 0); break;   //I90煞車 F2
		default:
			break;
		}
	}

	return CDialog::PreTranslateMessage(pMsg);
}

void SHHuangDlg::save_car_pos()
{


	if (PhotoCount)
	{
		Camera[0] = BinocularSLAM.p3dx_x.x;
		Camera[1] = BinocularSLAM.p3dx_x.y;
		Camera[2] = BinocularSLAM.p3dx_x.z;
		Camera[3] = BinocularSLAM.p3dx_x.x_dir;
		Camera[4] = BinocularSLAM.p3dx_x.y_dir;
		Camera[5] = BinocularSLAM.p3dx_x.z_dir;
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			Camera[i] = 0;
		}
	}

	CvPoint2D64f Pos;
	Pos.x = Camera[0];
	Pos.y = Camera[1];
	Path.push_back(Pos);


}


template<size_t LENGTH> void SHHuangDlg::SendSocketMessage(char(&data)[LENGTH]) {

	// Send Message
	if (fg_tcp_ip_connected) {
		m_tcp_socket.Send(data, LENGTH);
		ReportTCPStatus(TCPEvent::SEND_MESSAGE_SUCCESSFUL, CString(data));
	}
}

void SHHuangDlg::DoSocketConnect() {

	// Sync the Panel data to the model
	UpdateData(TRUE);

	// Read the TCP/IP address setting from User
	byte aIpAddressUnit[4];
	m_socket_ip_c.GetAddress(
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);
	CString aStrIpAddress;
	aStrIpAddress.Format(_T("%d.%d.%d.%d"),
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);

	// Create a TCP Socket for transfer Camera data
	// m_tcp_socket.Create(m_tcp_ip_port, 1, aStrIpAddress);
	if (!m_socket.Create()) {

		// If Socket Create Fail Report Message
		TCHAR szMsg[1024] = { 0 };
		wsprintf(szMsg, _T("create faild: %d"), m_socket.GetLastError());
		ReportSocketStatus(TCPEvent::CREATE_SOCKET_FAIL);
		AfxMessageBox(szMsg);
	}
	else {

		ReportSocketStatus(TCPEvent::CREATE_SOCKET_SUCCESSFUL);
		// Connect to the Server ( Raspberry Pi Server )
		fg_connected = m_socket.Connect(aStrIpAddress, m_socket_port);

		//fg_tcp_ip_read = true;
		//m_tcp_ip_IOHandle_thread = AfxBeginThread(TcpIODataHandler, LPVOID(this));

		//For Test
		m_socket.Send("Test from here", 14);
	}

	if (fg_connected)
		ReportSocketStatus(TCPEvent::CONNECT_SUCCESSFUL, aStrIpAddress);
	else
		ReportSocketStatus(TCPEvent::CONNECT_FAIL, aStrIpAddress);

}

void SHHuangDlg::DoSocketDisconnect() {
	// Setup Connect-staus flag
	fg_connected = false;
	//fg_tcp_ip_read = false;

	// Close the TCP/IP Socket
	m_socket.Close();

	// Report TCP/IP connect status
	CString tmp_log; tmp_log.Format(_T("I/O event: %s"), _T("Close Socket"));
	m_socket_log_c.AddString(tmp_log);
}

void SHHuangDlg::ReportSocketStatus(TCPEvent event_, CString &msg) {
	CString tmp_log;
	switch (event_) {
	case CREATE_SOCKET_SUCCESSFUL:
		tmp_log.Format(_T("I/O event: %s"),
			_T("Create Socket Successful"));
		break;
	case CREATE_SOCKET_FAIL:
		tmp_log.Format(_T("I/O event: %s"),
			_T("Create Socket Fail"));
		break;
	case CONNECT_SUCCESSFUL:
		tmp_log.Format(_T("I/O event: %s%s%s"),
			_T("Connect "), msg, _T(" Successful"));
		break;
	case CONNECT_FAIL:
		tmp_log.Format(_T("I/O event: %s%s%s"),
			_T("Connect "), msg, _T(" Fail"));
		break;
	case DISCONNECT:
		tmp_log.Format(_T("I/O event: %s"),
			_T("Disconnect"));
		break;
	case SEND_MESSAGE_SUCCESSFUL:
		tmp_log.Format(_T("I/O event: %s%s%s"),
			_T("Sent Message"), msg, _T("Successful"));
		break;
	case SENT_MESSAGE_FAIL:
		tmp_log.Format(_T("I/O event: %s%s%s"),
			_T("Sent Message"), msg, _T("Fail"));
		break;
	}
	m_socket_log_c.AddString(tmp_log);
}

void SHHuangDlg::I90_PWM_send(int L_PWM, int R_PWM)
{
	int L, R, mid = 16384; //0~32767

	if (R_PWM > 0)
		R_PWM = R_PWM + c_Synchronous;

	if (R_PWM < 0)
	{
		L_PWM = L_PWM - c_Synchronous;
		R_PWM = R_PWM;
	}

	//L.R.轉換
	L = mid + L_PWM;
	R = mid - R_PWM;



	I90_PWM_control[7] = R;
	I90_PWM_control[8] = R >> 8;
	I90_PWM_control[10] = L;
	I90_PWM_control[11] = L >> 8;
	I90_PWM_control[(I90_PWM_control[5] + 6)] = checksun(2, (I90_PWM_control[5] + 5));


	//I90rs232.Open(7,115200);
	Connect_I90.SendData(I90_PWM_control, sizeof(I90_PWM_control));
}

unsigned char SHHuangDlg::checksun(int nStar, int nEnd)
{
	unsigned char shift_reg, sr_lsb, data_bit, v, fb_bit;
	int i, j;

	shift_reg = 0;
	for (i = nStar; i <= (nEnd); i++)
	{
		v = (unsigned char)(I90_PWM_control[i] & 0x0000ffff);
		for (j = 0; j < 8; j++)
		{
			data_bit = v & 0x01;  // isolate least sign bit
			sr_lsb = shift_reg & 0x01;
			fb_bit = (data_bit ^ sr_lsb) & 0x01;  //calculate the feed back bit
			shift_reg = shift_reg >> 1;
			if (fb_bit == 1)
			{
				shift_reg = shift_reg ^ 0x8c;
			}
			v = v >> 1;
		}
	}
	return shift_reg;
}



void CRSocket::OnReceive(int nErrorCode) {

	static unsigned int counter(0);

	if (0 == nErrorCode) {
		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(SIZE_POSE_DATA * sizeof(double)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {
			// Determine Socket State
			nRead = Receive((byte*)&m_pose_data + cbDataReceived, cbLeft);

			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					AfxMessageBox(_T("Error occurred"));
					Close();
				}
				break;
			}
			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);

	}

	Laser_pos[0] = m_pose_data[0];
	Laser_pos[1] = m_pose_data[1];
	Laser_pos[2] = m_pose_data[2];


	CSocket::OnReceive(nErrorCode);
}

BOOL CRSocket::OnMessagePending()
{
	return 0;
}

void CRSocket::registerParent(CWnd* _parent)
{
	m_parent = _parent;
}




