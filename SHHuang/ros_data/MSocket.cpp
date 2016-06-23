#include "stdafx.h"
#include "MSocket.h"
#include "MapDataReceiverDlg.h"
#include "resource.h"


CMSocket::CMSocket() {
	m_map_data = new MapData;
	int buffer_size = 1024 * 1024 * 16;
	SetSockOpt(SO_RCVBUF, &buffer_size, sizeof(int));
}


CMSocket::~CMSocket() {
	delete m_map_data;
}

void CMSocket::OnReceive(int nErrorCode) {

	static unsigned int counter(0);
	MapDataPartPtr ptrMap(new MapDataPart);

	if (0 == nErrorCode) {
		
		static int i = 0;
		i++;
		
		int nRead(0);

		volatile int cbLeft(sizeof(MapDataPart)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {

			
			// Determine Socket State
			nRead = Receive(ptrMap.get() + cbDataReceived, cbLeft);

			m_map_parts.push_back(ptrMap);

#ifdef _SHOW_SOCKET_DEBUG

			/****** Print Socket status inorder to debug ****/
			CString msg;
			msg.Format(_T("Stamp: %d Rec count: %d, Left count: %d"), ptrMap->info.part_stamp, nRead, cbLeft);
			m_parent->SetDlgItemTextW(IDC_RECE_STATUS, msg);
			m_parent->UpdateData(false);
			/**********************************************/

			if(cbLeft < 0) AfxMessageBox(_T("WTF occurred"));
#endif // _SHOW_SOCKET_DEBUG

			
			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					((CMapDataReceiverDlg*)m_parent)->DoMapSocketConnect();
					return;
				}
				break;
			}


			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);

		

	}

	if (ptrMap->info.part_stamp == (MAP_PART_COUNT - 1)*MAP_PER_CUT_SIZE) {
		ptrMap.reset();
		assembleData();
		// Show the data to View module

		showData();
	}

	//if (m_map_parts.size() == MAP_PART_COUNT) {
	//	assembleData();
	//	// Show the data to View module
	//	showData();
	//}



	CSocket::OnReceive(nErrorCode);
}

BOOL CMSocket::OnMessagePending() {
	

	return 0;
}

void CMSocket::showData() {


	// ***********************************
	// Show the Map data to User
	// ***********************************
	

	CDC *pDC = m_parent->GetDlgItem(IDC_POS_SHOW)->GetDC();
	pDC->SetBkMode(TRANSPARENT);
	CStatic* pPic = (CStatic*)m_parent->GetDlgItem(IDC_POS_SHOW);



	// Transform the raw data to BITMAT format

	//create a memory dc
	CDC memDC;
	memDC.CreateCompatibleDC(pDC);

	//Create a memory bitmap
	CBitmap newbmp;
	
	uint8_t buffer_map[400 * 400 * 4];

	for (int i = 0; i < 400 * 400 * 4; i+=4) {
		buffer_map[i] = m_map_data->data[i / 4];
		buffer_map[i+1] = m_map_data->data[i / 4];
		buffer_map[i+2] = m_map_data->data[i / 4];
		buffer_map[i+3] = 255;
	}
	newbmp.CreateCompatibleBitmap(pDC, 400, 400);

	newbmp.SetBitmapBits(sizeof(buffer_map), buffer_map);

	//select the bitmap in the memory dc
	CBitmap *pOldBitmap = memDC.SelectObject(&newbmp);

	memDC.SetBkColor(RGB(255,255, 255));

	//blit from memory dc into screen
	pDC->BitBlt(0, 0, 400, 400, &memDC, 0, 0, SRCCOPY);


	//select old bitmap back into the memory dc
	memDC.SelectObject(pOldBitmap);



	//BITMAP bitmap;
	//HBITMAP hMap;
	//hMap = CreateBitmap(MAP_SIZE_X, MAP_SIZE_X, 4,
	//32, m_map_data->data);

	//// Print the map data to the screen
	//pPic->ModifyStyle(0xf, SS_BITMAP | SS_CENTERIMAGE);
	//pPic->SetBitmap(hMap);

	CFont font;


	font.CreateFont(
		16, // nHeight 
		0, // nWidth 
		0, // nEscapement 
		0, // nOrientation 
		FW_NORMAL, // nWeight 
		FALSE, // bItalic 
		FALSE, // bUnderline 
		0, // cStrikeOut 
		ANSI_CHARSET, // nCharSet 
		OUT_DEFAULT_PRECIS, // nOutPrecision 
		CLIP_DEFAULT_PRECIS, // nClipPrecision 
		DEFAULT_QUALITY, // nQuality 
		DEFAULT_PITCH | FF_SWISS,
		_T("Arial") // nPitchAndFamily Arial 
		);
	pDC->SelectObject(&font);


	char buffer[100];
	sprintf_s(buffer, 50, "Resolution: %5.2f",
		m_map_data->info.res);
	pDC->DrawText(CString(buffer), CRect(5, 0, 200, 20), DT_SINGLELINE | DT_LEFT | DT_VCENTER);

	sprintf_s(buffer, 50, "Map Height: %4d",
		m_map_data->info.height);
	pDC->DrawText(CString(buffer), CRect(5, 20, 200, 40), DT_SINGLELINE | DT_LEFT | DT_VCENTER);

	sprintf_s(buffer, 50, "Map Width: %4d",
		m_map_data->info.width);
	pDC->DrawText(CString(buffer), CRect(5, 40, 200, 60), DT_SINGLELINE | DT_LEFT | DT_VCENTER);

	sprintf_s(buffer, 50, "Origin : %5.2f, %5.2f",
		m_map_data->info.origin_x, m_map_data->info.origin_y);
	pDC->DrawText(CString(buffer), CRect(5, 60, 200, 80), DT_SINGLELINE | DT_LEFT | DT_VCENTER);

	sprintf_s(buffer, 50, "Rotation: %5.2f",
		m_map_data->info.origin_yaw);
	pDC->DrawText(CString(buffer), CRect(5, 80, 200, 100), DT_SINGLELINE | DT_LEFT | DT_VCENTER);

	sprintf_s(buffer, 50, "Map Count: %5d",
		m_map_data->info.map_stamp);
	pDC->DrawText(CString(buffer), CRect(5, 100, 200, 120), DT_SINGLELINE | DT_LEFT | DT_VCENTER);
}

void CMSocket::assembleData() {

	// Copy the Meta data
	m_map_data->info.map_stamp = m_map_parts.at(0)->info.map_stamp;
	m_map_data->info.res =  m_map_parts.at(0)->info.res;
	m_map_data->info.height = m_map_parts.at(0)->info.height;
	m_map_data->info.width = m_map_parts.at(0)->info.width;
	m_map_data->info.origin_x = m_map_parts.at(0)->info.origin_x;
	m_map_data->info.origin_y = m_map_parts.at(0)->info.origin_y;
	m_map_data->info.origin_yaw = m_map_parts.at(0)->info.origin_yaw;

	// Copy the map data
	int i(0);
	//if (m_map_parts.size() != MAP_PART_COUNT) {
	//	m_map_parts.clear();
	//	return;
	//}
	for (MapDataPartArray::iterator it = m_map_parts.begin();
					it != m_map_parts.end();
						it++, i++) {

		// Get each map part data pointer
		uint8_t* ptrData = it->get()->data;
		size_t offset = it->get()->info.part_stamp;
		if (!(offset > (MAP_PART_COUNT - 1)*MAP_PER_CUT_SIZE)) {
			memcpy(m_map_data->data + offset, ptrData, MAP_PER_CUT_SIZE);
		}
	}

	// Clear all data
	m_map_parts.clear();

}

void CMSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}
