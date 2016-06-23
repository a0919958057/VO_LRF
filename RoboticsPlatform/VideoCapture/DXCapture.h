//	DxCapture.h - Definition
//
//	Copyright (C) 2006-2008 Robot Vision Lab, Tamkang University 

#ifndef _DxCapture_H_
#define _DxCapture_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#define __D3DRM_H__
#include "tchar.h"
#include "qedit.h"
#include <dshow.h>
#include <cv.h>
#include <cxcore.h>
#include <windows.h>

//	It have to include "strmiids.lib" before using Direct Show
#pragma comment(lib,"strmiids.lib")

/*	-----------------------------------------------------------------
 *	Variable
 *	-----------------------------------------------------------------*/
#define WM_GRAPHNOTIFY		WM_APP+1

#ifndef SAFE_RELEASE
#define SAFE_RELEASE(x) \
	if( NULL != x )		\
	{					\
		x->Release( );	\
		x = NULL;		\
	}					
#endif

/*	-----------------------------------------------------------------
 *	�x�s�˸m������Ƶ��c
 *	-----------------------------------------------------------------*/
typedef struct __CapStuff
{
	unsigned int index;													// Device index
	char		 *deviceName;											// Device name
	IMoniker	 *rgpmVideoMenu;										// Device menu
} CAPSTUFF;

/*	-----------------------------------------------------------------
 *	�v���^�¬�����Ƶ��c
 *	-----------------------------------------------------------------*/
typedef struct __CallBackInfo 
{
    double dblSampleTime;
    long lBufferSize;													// Buffer size
    BYTE *pBuffer;														// Video stream
} CALLBACKINFO;

/*	-----------------------------------------------------------------
 *	CSampleGrabberCB Class
 *	  for the callback of "ISampleGrabber" we have to implement the 
 *	class when the member function "SetCallback" called, and we add 
 *	some parameter to save the information when sampling							
 *	-----------------------------------------------------------------*/	
class CSampleGrabberCB : public ISampleGrabberCB 
{
	public:
		CSampleGrabberCB();
		virtual ~CSampleGrabberCB();

	public:
		BYTE *cbinfo();
		//void cbinfo( BYTE **pStream);

	private:		
		CALLBACKINFO cb;

		STDMETHODIMP_(ULONG) AddRef();
		STDMETHODIMP_(ULONG) Release();

		// fake out any COM QI'ing
		STDMETHODIMP QueryInterface(
			REFIID	riid,
			void	**ppv);

		// we don't implement this interface for this example
		STDMETHODIMP SampleCB(
			double			SampleTime,
			IMediaSample	*pSample);

		// �v���H Bitmap �榡�x�s�b pBuffer
		STDMETHODIMP BufferCB( 
			double	dblSampleTime,
			BYTE	*pBuffer,
			long	lBufferSize);
};

/*	-----------------------------------------------------------------
 *	VideoCapture class�G	
 *	-----------------------------------------------------------------*/
class VideoCapture 
{
	//	Constructor
	public:
		VideoCapture();
		virtual ~VideoCapture();

	public:	
		bool IsInitial;													//	Initial flag
		
		HRESULT CcdInitial( 
			HWND hwnd = 0,												//	Windows point
			int DeviceIndex = 0);										//	Device index

		HRESULT Capture();												//	Start garbbing image
		void StopCapture();												//	Stop garbbing image

		void ConfigVideoPin();											//	Video format
		void ConfigVideoFilter();										//	Video setting
		
		void CleanUp();													//	Clean all point
		
		void GetOneImage( BYTE *pImage );								//	Capture one image
		IplImage *GetOneImage();										//	Capture one image
		
		char* Get_DeviceName();											//	Get device name
		unsigned int Get_DeviceID();									//	Get device index
		
		long Get_Width();												//	Get video width
		long Get_Height();												//	Get video height

		int GetCaptureCount();
		int GetDeviceName(int nCamID, char* sName, int nBufferSize);
		
	protected:
		// Search capture device
		HRESULT GetCaptureDevice();

		HRESULT GetInterfaces();
		void CloseInterface();

		HRESULT GetPin(
			IBaseFilter		*pFilter,
			PIN_DIRECTION	PinDir,
			IPin			**ppPin);

		// Set up video window
		HRESULT SetupVideoWindow();
		void ResizeVideoWindow();
		
		// Save image to pointer "pImage"
		HRESULT SaveImage( BYTE *pImage );

	#ifdef _DEBUG
		// Return error message
		void ErrMsg(
			char *szFormat,
			...);
	#endif

	private:
		bool IsCapture;													//	Capture flag
		long lWidth;													//	Video width
		long lHeight;													//	Video height

		IplImage *pImage;
		HWND m_hwnd;
		CAPSTUFF gCapInfo;
		CSampleGrabberCB mCB;

		IBaseFilter				*pSrc;
		ICaptureGraphBuilder2	*pBuilder;
		IGraphBuilder			*pGraph;
		IMediaFilter			*pMediaFilter;
		IMediaControl			*pMediaControl;
		IMediaEventEx			*pMediaEvent;
		IVideoWindow			*pVidWin;
		ISampleGrabber			*pGrabber;
};

#endif