//	DxCapture.h - Definition
//
//	Copyright (C) 2006-2008 Robot Vision Lab, Tamkang University 

#include "DxCapture.h"
#include <atlbase.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

/*	-----------------------------------------------------------------
 *	CSampleGrabberCB Class
 *	-----------------------------------------------------------------*/
CSampleGrabberCB::CSampleGrabberCB()
{
}

CSampleGrabberCB::~CSampleGrabberCB()
{
	//if( cb.pBuffer )
	//{
		//free( cb.pBuffer);
	//}
}

BYTE *CSampleGrabberCB::cbinfo()
{
	return cb.pBuffer;
}

/*
void CSampleGrabberCB::cbinfo(
	BYTE **pStream)
{
	if( *cb.pBuffer)
	{
		*pStream = cb.pBuffer;
	}
}
*/

STDMETHODIMP_(ULONG) CSampleGrabberCB::AddRef()
{ 
	return 2; 
}

STDMETHODIMP_(ULONG) CSampleGrabberCB::Release()
{
	return 1;
}

STDMETHODIMP CSampleGrabberCB::QueryInterface(
	REFIID riid,
	void ** ppv)
{
	if( riid == IID_ISampleGrabberCB || riid == IID_IUnknown ) 
	{
		*ppv = (void *) static_cast<ISampleGrabberCB*> ( this );
		return NOERROR;
	}    
	return E_NOINTERFACE;
}

STDMETHODIMP CSampleGrabberCB::SampleCB( 
				double SampleTime,
				IMediaSample * pSample)
{
	return 0;
}

/*	-----------------------------------------------------------------
 *	Saving the callback information
 *	-----------------------------------------------------------------*/
STDMETHODIMP CSampleGrabberCB::BufferCB( 
	double dblSampleTime,
	BYTE pBuffer[],
	long lBufferSize)
{
	/*	-------------------------------------------------------------
	 *	Since we can't access Windows API functions in this callback, 
	 *  just copy the bitmap data to a structure for later reference.
	 *	-------------------------------------------------------------*/
	cb.dblSampleTime = dblSampleTime;
	cb.lBufferSize = lBufferSize;

	/*	-------------------------------------------------------------
	 *	If we haven't yet allocated the data buffer, do it now.
     *	Just allocate what we need to store the new bitmap.
	 *	-------------------------------------------------------------*/
	//if( cb.pBuffer )
	//{
	//	free( cb.pBuffer);
	//}
	cb.pBuffer = pBuffer;
	//cb.pBuffer = (unsigned char *)malloc(lBufferSize);
	
	/*	-------------------------------------------------------------
	 *	Copy the bitmap data into our buffer
	 *	-------------------------------------------------------------*/
	//if(cb.pBuffer)
	//{
	//	memcpy(
	//		cb.pBuffer,
	//		pBuffer, 
	//		lBufferSize);
	//}

	/*	-------------------------------------------------------------
	 *	Post a message to our application, telling it to come back
     *	and write the saved data to a bitmap file on the user's disk.
	 *	-------------------------------------------------------------*/
	return 0;
}


/*	-----------------------------------------------------------------
 *	VideoCapture class
 *	-----------------------------------------------------------------*/
VideoCapture::VideoCapture()
{
	IsInitial = false;
	IsCapture = false;
}

VideoCapture::~VideoCapture()
{
	CleanUp();
	
	// Release COM
    //CoUninitialize();
}

/*	------------------------------------------------------------------
 *	public
 *	------------------------------------------------------------------ */
// For Initial video capture
HRESULT VideoCapture::CcdInitial(
	HWND hwnd/* = 0*/,
	int DeviceIndex /*= 0*/)
{
	HRESULT hr = NULL;

	m_hwnd = hwnd;
	gCapInfo.index = DeviceIndex;

	//	Try if it had been initialized
	if( IsInitial )
	{
		CleanUp();
	}

	// Initial Componet object model
	hr = CoInitialize(NULL);
		
	if( FAILED(hr) ) 
	{
		#ifdef _DEBUG
			ErrMsg( TEXT("Initial COM failed!\r\n") );   
		#endif

		return hr;
	}	
	
	hr = GetCaptureDevice();
    
	if( hr != NOERROR )
    {
        // Don't display a message because GetCaptureDevice function will handle it
        return hr;
    }

	IsInitial = true;

	return 1;
}

// Capture video
HRESULT VideoCapture::Capture()
{
	HRESULT hr;

	if(IsCapture)
	{
		IsCapture = false;
		CleanUp();
		Capture();
		return 0;
	}

    hr = GetInterfaces();
    
	if( FAILED(hr) )
    {
		#ifdef _DEBUG
			ErrMsg(TEXT("Failed to get video interfaces! hr=ox%x"), hr);
		#endif

        return hr;
    }

	hr = GetCaptureDevice();
    
	if( hr != NOERROR )
    {
        // Don't display a message because GetCaptureDevice function will handle it
        return hr;
    }

	// Attach the filter graph to the capture graph
    hr = pBuilder->SetFiltergraph( pGraph );
    
	if( FAILED(hr) )
    {
        return hr;
    }

	IMoniker *pmVideoMenu;	

	pmVideoMenu = gCapInfo.rgpmVideoMenu;	
	
	if(!pmVideoMenu)
	{
		return E_FAIL;
	}

	// Function "GetCaptureDevice" have already get the device
	// Bind Moniker to a filter object
	hr = pmVideoMenu->BindToObject(
			0,
			0,
			IID_IBaseFilter,
			(void**)&pSrc);
	
	if( FAILED(hr) )
    {		
		return hr;
    }

    // Add Capture filter to our graph.
    hr = pGraph->AddFilter(
			pSrc,
			L"Video Capture");
    
	if( FAILED(hr) )
    {
        pSrc->Release();
        return hr;
    }

	//-------------------------------------------
	ConfigVideoPin();

	pMediaControl->Run();
    pMediaFilter->SetSyncSource(NULL); // Turn off the reference clock.
    
	/*	-------------------------------------------------------------
	 *	  set up the callback of pGrabber, all infomation will save 
	 *	in member data of "CSampleGrabberCB", include "sampling time",
	 *	"buffer", and "buffer size" and data will auto reflash in each
	 *	sampling time				
	 *	-------------------------------------------------------------*/
	hr = pGrabber->SetOneShot(false);
    
	if (FAILED(hr))
	{
        return hr;
	}
	
	hr = pGrabber->SetBufferSamples(false);
    
	if (FAILED(hr))
	{
        return hr;
	}
    
	hr = pGrabber->SetCallback(&mCB,1);
    
	if ( FAILED(hr) )
	{
        return hr;
	}

    pSrc->Release();
    pGrabber->Release();

	IsCapture = true;

    return S_OK;
}

//	Stop capture video
void VideoCapture::StopCapture()
{
	CloseInterface();
}

// Set media type
void VideoCapture::ConfigVideoPin()
{
	if(IsCapture)
	{
		IsCapture = false;
		CleanUp();
		Capture();
		return ;
	}

	HRESULT hr;
	IAMStreamConfig *pVSC;
	
	hr = pBuilder->FindInterface(
			&PIN_CATEGORY_CAPTURE,
			&MEDIATYPE_Interleaved, 
			pSrc,
			IID_IAMStreamConfig,
			(void **)&pVSC);

	if( hr != NOERROR )
	{
		hr = pBuilder->FindInterface(
				&PIN_CATEGORY_CAPTURE,
				&MEDIATYPE_Video, 
				pSrc,
				IID_IAMStreamConfig, 
				(void **)&pVSC);
	}

	ISpecifyPropertyPages *pSpec;
	CAUUID cauuid;

	hr = pVSC->QueryInterface(
			IID_ISpecifyPropertyPages,
			(void **)&pSpec);

	if( hr == S_OK )
	{
		hr = pSpec->GetPages(&cauuid);

	    hr = OleCreatePropertyFrame(
				/*GetActiveWindow()*/NULL,
				30,
				30,
				NULL,
				1,
				(IUnknown **)&pVSC,
				cauuid.cElems,
				(GUID *)cauuid.pElems,
				0,
				0,
				NULL);

		CoTaskMemFree(cauuid.pElems);

		pSpec->Release();
	}	

	//-------------------------------------------
	IBaseFilter *pF = NULL;
    
	hr = CoCreateInstance(
			CLSID_SampleGrabber,
			NULL,
			CLSCTX_INPROC_SERVER,
			IID_IBaseFilter,
			reinterpret_cast<void**>(&pF));

	//-------------------------------------------
	hr = pF->QueryInterface(
			IID_ISampleGrabber,
			reinterpret_cast<void**>(&pGrabber));

	//-------------------------------------------
    hr = pGraph->AddFilter(
			pF,
			L"SampleGrabber");

    // Set media type
    AM_MEDIA_TYPE mt;
    ZeroMemory(
		&mt,
		sizeof(AM_MEDIA_TYPE));

    mt.majortype = MEDIATYPE_Video;
	
	//-------------------------------------------
	HDC hdc;
    int iBitDepth;
	
	hdc = GetDC(NULL);
	iBitDepth = GetDeviceCaps( hdc, BITSPIXEL);
    ReleaseDC( NULL, hdc);
    
	switch(iBitDepth)
    {
        case 8:
            mt.subtype = MEDIASUBTYPE_RGB8;
        break;

        case 16:
            mt.subtype = MEDIASUBTYPE_RGB555;
        break;

        case 24:
            mt.subtype = MEDIASUBTYPE_RGB24;
        break;

        case 32:
            mt.subtype = MEDIASUBTYPE_RGB32;
        break;

        default:
           // return E_FAIL;
			;
    }

    hr = pGrabber->SetMediaType(&mt);

	//-------------------------------------------
    IPin *pCapOut;
    
	GetPin( 
		pSrc,
		PINDIR_OUTPUT,
		&pCapOut);
    
	IPin *pGrabIn; 
	
	GetPin( 
		pF,
		PINDIR_INPUT,
		&pGrabIn);
    
	hr = pGraph->Connect(
			pCapOut,
			pGrabIn );

    IPin *pGrabOut;
    
	GetPin( 
		pF,
		PINDIR_OUTPUT,
		&pGrabOut);

	// Output Video to window
	hr = pGraph->Render( pGrabOut );
    
	if( FAILED(hr) )
    {
        pGrabOut->Release();
    }

	hr = pGrabber->GetConnectedMediaType( &mt );
    
	//	Get Information from video stream
    VIDEOINFOHEADER *vih = (VIDEOINFOHEADER*) mt.pbFormat;
    lWidth  = vih->bmiHeader.biWidth;
    lHeight = vih->bmiHeader.biHeight;

	//	Set preview window
	if( m_hwnd )
	{
		pBuilder->RenderStream(
			&PIN_CATEGORY_PREVIEW,
			&MEDIATYPE_Video,
			pF,
			NULL,
			NULL);
	
		SetupVideoWindow();
	}

	pF->Release();
	pVSC->Release();

	return ;
}

void VideoCapture::ConfigVideoFilter()
{
	if( IsInitial )
	{
	HRESULT hr;
	IAMVideoProcAmp *pVC;

	hr = pBuilder->FindInterface(
			&PIN_CATEGORY_PREVIEW,
			&MEDIATYPE_Interleaved, 
			pSrc,
			IID_IAMVideoProcAmp,
			(void **)&pVC);

	if( hr != NOERROR )
	{
		hr = pBuilder->FindInterface(
				&PIN_CATEGORY_PREVIEW,
				&MEDIATYPE_Video, 
				pSrc,
				IID_IAMVideoProcAmp, 
				(void **)&pVC);
	}

	ISpecifyPropertyPages *pSpec;
	CAUUID cauuid;
	
	hr = pVC->QueryInterface(
			IID_ISpecifyPropertyPages,
			(void **)&pSpec);

	if( hr == S_OK )
	{
		hr = pSpec->GetPages( &cauuid );

		hr = OleCreatePropertyFrame(
				/*GetActiveWindow()*/NULL,
				30,
				30,
				NULL,
				1,
				(IUnknown **)&pVC,
				cauuid.cElems,
				(GUID *)cauuid.pElems,
				0,
				0,
				NULL);

		CoTaskMemFree(cauuid.pElems);

		pSpec->Release();
	}	

	pVC->Release();
	}
	return ;
}

// 睲埃夹
void VideoCapture::CleanUp()
{
	SAFE_RELEASE(pSrc);

	SAFE_RELEASE(pGraph);
	SAFE_RELEASE(pBuilder);
	
	if(pVidWin)
	{
		pVidWin->put_Visible(OAFALSE);
		pVidWin->put_Owner(NULL);
	}
	SAFE_RELEASE(pVidWin);

	SAFE_RELEASE(pMediaEvent);
	SAFE_RELEASE(pMediaControl);
	SAFE_RELEASE(pMediaFilter);
	
	SAFE_RELEASE(pGrabber);
	
	IsInitial = false;
	IsCapture = false;
}

// Save video to pointer " pImage "
void VideoCapture::GetOneImage( BYTE *pImage )
{
	SaveImage( pImage);
}

//	Get device Name
char* VideoCapture::Get_DeviceName()
{
	return gCapInfo.deviceName;
}

//	Get device Index
unsigned int VideoCapture::Get_DeviceID()
{
	return gCapInfo.index;
}

//	Get video width
long VideoCapture::Get_Width()
{
	return lWidth;
}

//	Get video height
long VideoCapture::Get_Height()
{
	return lHeight;
}

int VideoCapture::GetCaptureCount()
{

	int count = 0;
 	CoInitialize(NULL);

   // enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
									IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
        &pEm, 0);
    if (hr != NOERROR) 
	{
		return count;
    }

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while(hr = pEm->Next(1, &pM, &cFetched), hr==S_OK)
    {
		count++;
    }

	pCreateDevEnum = NULL;
	pEm = NULL;
	return count;
}

int VideoCapture::GetDeviceName(int nCamID, char* sName, int nBufferSize)
{
	int count = 0;
 	CoInitialize(NULL);

   // enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
									IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
        &pEm, 0);
    if (hr != NOERROR) return 0;


    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while(hr = pEm->Next(1, &pM, &cFetched), hr==S_OK)
    {
		if (count == nCamID)
		{
			IPropertyBag *pBag=0;
			hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
			if(SUCCEEDED(hr))
			{
				VARIANT var;
				var.vt = VT_BSTR;
				hr = pBag->Read(L"FriendlyName", &var, NULL); //还有其他属性,像描述信息等等...
	            if(hr == NOERROR)
		        {
			        //获取设备名称			
					WideCharToMultiByte(CP_ACP,0,var.bstrVal,-1,sName, nBufferSize ,"",NULL);
	                SysFreeString(var.bstrVal);
		        }
			    pBag->Release();
			}
			pM->Release();

			break;
		}
		count++;
    }

	pCreateDevEnum = NULL;
	pEm = NULL;

	return 1;
}

/*	------------------------------------------------------------------
 *	private
 *	------------------------------------------------------------------ */
// Search capture device
HRESULT VideoCapture::GetCaptureDevice()
{
	USES_CONVERSION;

	UINT    uIndex = 0;
    HRESULT hr;

	SAFE_RELEASE(gCapInfo.rgpmVideoMenu);
	
    //-------------------------------------------
    ICreateDevEnum *pDevEnum= NULL;

    hr = CoCreateInstance(
			CLSID_SystemDeviceEnum,
			NULL,
			CLSCTX_INPROC_SERVER,
			IID_ICreateDevEnum,
			(void**)&pDevEnum);

    if( hr != NOERROR )
	{
        return hr;
    }

    // Enumerate all capture device
    IEnumMoniker *pClassEnum = NULL;

    hr = pDevEnum->CreateClassEnumerator(
			CLSID_VideoInputDeviceCategory, 
			&pClassEnum,
			0);

    if( hr != NOERROR )
    {
		return hr;
    }

	pClassEnum->Reset();

    ULONG cFetched;
    IMoniker *pM;
	
	while( hr = pClassEnum->Next(1, &pM, &cFetched), hr==S_OK)
	{
        IPropertyBag *pBag;

        hr = pM->BindToStorage(
				0,
				0,
				IID_IPropertyBag,
				(void **)&pBag);
        
		if( SUCCEEDED(hr) && (uIndex == (UINT)gCapInfo.index) )
		{
            VARIANT var;
            var.vt = VT_BSTR;

            hr = pBag->Read(
					L"FriendlyName",
					&var,
					NULL);

			// Record device name
			gCapInfo.deviceName = (char*)malloc(strlen(W2T(var.bstrVal)));
			strcpy(gCapInfo.deviceName, W2T(var.bstrVal));
                
			SysFreeString(var.bstrVal);

			// Save the device IMoniker
            gCapInfo.rgpmVideoMenu = pM;
            pM->AddRef();
        
			SAFE_RELEASE(pBag);
        }

		SAFE_RELEASE(pM);
        
        uIndex++;
    }

    pClassEnum->Release();
	pDevEnum->Release();
	
	return S_OK;
}

// Get onterface
HRESULT VideoCapture::GetInterfaces()
{
    HRESULT hr;

    hr = CoCreateInstance (
			CLSID_FilterGraph,
			NULL,
			CLSCTX_INPROC,
			IID_IGraphBuilder,
			(void **) &pGraph);

    if( FAILED(hr) )
	{
		return hr;
	}

    // Create the capture graph builder
    hr = CoCreateInstance(
			CLSID_CaptureGraphBuilder2,
			NULL,
			CLSCTX_INPROC,
			IID_ICaptureGraphBuilder2,
			(void **) &pBuilder);

    if( FAILED(hr) )
	{
        return hr;
	}

    // Obtain interfaces for media control and Video Window
    hr = pGraph->QueryInterface(
			IID_IMediaControl,
			(LPVOID *) &pMediaControl);

    if( FAILED(hr) )
	{
        return hr;
	}

	//-------------------------------------------
    hr = pGraph->QueryInterface(
			IID_IVideoWindow,
			(LPVOID *) &pVidWin);

    if( FAILED(hr) )
	{
		return hr;
	}

	//-------------------------------------------
	hr = pGraph->QueryInterface(
			IID_IMediaEvent,
			(LPVOID *) &pMediaEvent);

    if( FAILED(hr) )
	{
        return hr;
	}

	//-------------------------------------------
    hr = pGraph->QueryInterface(
			IID_IMediaFilter,
			(LPVOID *) &pMediaFilter);

    if( FAILED(hr) )
	{
        return hr;
	}

	//-------------------------------------------
	hr = pMediaEvent->SetNotifyWindow( 
			(OAHWND)m_hwnd,
			WM_GRAPHNOTIFY,
			0);

    return hr;
}

void VideoCapture::CloseInterface()
{
	//	Stop previewing data
	if(pMediaControl)
	{
		pMediaControl->StopWhenReady();
	}
	SAFE_RELEASE(pMediaControl);


	//	Stop receiving events
	if(pMediaEvent)
	{
		pMediaEvent->SetNotifyWindow(
			NULL,
			WM_GRAPHNOTIFY,
			0);
	}
	SAFE_RELEASE(pMediaEvent);


	if(pVidWin)
	{
		pVidWin->put_Visible(OAFALSE);
		pVidWin->put_Owner(NULL);
	}
	SAFE_RELEASE(pVidWin);
	
	SAFE_RELEASE(pBuilder);
}

// Make pin
HRESULT VideoCapture::GetPin(
	IBaseFilter *pFilter,
	PIN_DIRECTION PinDir,
	IPin **ppPin)
{
    IEnumPins  *pEnum;
    IPin       *pPin;

    pFilter->EnumPins(&pEnum);

    while( pEnum->Next(1, &pPin, 0) == S_OK )
    {
        PIN_DIRECTION PinDirThis;
        
		pPin->QueryDirection( &PinDirThis );

        if (PinDir == PinDirThis)
        {
            pEnum->Release();
            *ppPin = pPin;

            return S_OK;
        }

        pPin->Release();
    }

    pEnum->Release();

    return S_OK;
}

HRESULT VideoCapture::SetupVideoWindow()
{
	HRESULT hr;
	
	//	Set the video to be a child of te main window
	hr = pVidWin->put_Owner((OAHWND)m_hwnd);
	if(FAILED(hr))
	{
		return hr;
	}

	//	Set video window style
	hr = pVidWin->put_WindowStyle( WS_CHILD | WS_CLIPCHILDREN);
	if(FAILED(hr))
	{
		return hr;
	}

	hr = pVidWin->put_Width(lWidth);
	hr = pVidWin->put_Height(lHeight);

	ResizeVideoWindow();

	//	Make the video window visable, now that it is properly positioned
	hr = pVidWin->put_Visible(OATRUE);
	if(FAILED(hr))
	{
		return hr;
	}

	hr = pVidWin->put_MessageDrain((OAHWND)m_hwnd);

	return hr;
}

void VideoCapture::ResizeVideoWindow()
{	
	if(pVidWin)
	{
		RECT rc;

		::GetClientRect( m_hwnd, &rc);
		pVidWin->SetWindowPosition( 0, 0, rc.right, rc.bottom);
	}
}

// Save video stream to pointer " pImage "
HRESULT VideoCapture::SaveImage(
	BYTE		*pImage)
{	
	HRESULT hr;

    hr = pGrabber->SetOneShot(false);
    
	if( FAILED(hr) )
	{
        return hr;
	}

    hr = pGrabber->SetBufferSamples(false);
    
	if( FAILED(hr) )
	{
        return hr;
	}

    hr = pGrabber->SetCallback(&mCB,1);
    
	if( FAILED(hr) )
	{
        return hr;
	}

	// Get video stream pointer from callback
	BYTE *pStream = ( BYTE* )malloc( 4*lWidth*lHeight*sizeof(BYTE) );
	memcpy( pStream, mCB.cbinfo(), 4*lWidth*lHeight*sizeof(BYTE) );
	int index = 0, Vsize = lWidth*lHeight;
	for( index = 0 ; index < Vsize ; index++ )
	{
		memcpy( pImage+index*3, pStream+index*4, 3*sizeof(char) );
	}
	free(pStream);

    return hr;
}

#ifdef _DEBUG
void VideoCapture::ErrMsg( 
	char *szFormat,
	...)
{
    TCHAR szBuffer[512];

    va_list pArgList;
    va_start( pArgList, szFormat);
    _vstprintf( szBuffer, szFormat, pArgList);
    va_end(pArgList);

    MessageBox( 
		NULL,
		szBuffer,
		TEXT("DirectX Capture Message"), 
		MB_OK | MB_ICONWARNING
			  );
}
#endif

IplImage *VideoCapture::GetOneImage()
{
	pGrabber->SetOneShot(false);
    pGrabber->SetBufferSamples(false);
    pGrabber->SetCallback(&mCB,1);

	// Get video stream pointer from callback
	int index = 0, Vsize = lWidth*lHeight;


	pImage = cvCreateImage( cvSize( lWidth, lHeight ), IPL_DEPTH_8U, 3 );

	BYTE *pStream ;
	pStream = mCB.cbinfo();

	for( index = 0 ; index < Vsize ; index++ )
	{
		memcpy( pImage->imageData+index*3, pStream+index*4, 3*sizeof(char) );
	}

	cvFlip(pImage);

	return pImage;
}