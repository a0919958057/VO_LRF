#include "stdafx.h"
#include "RSocket.h"
#include "MapDataReceiverDlg.h"
#include "resource.h"


CRSocket::CRSocket() {}


CRSocket::~CRSocket() {}

void CRSocket::OnReceive(int nErrorCode) {
	if (0 == nErrorCode) {
		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(SIZE_LRF_DATA*4); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive((byte*)&m_lrf_data + cbDataReceived, cbLeft);

			//nRead = Receive(this->image_buffer, 1460);
			//memcpy_s(m_remote_image.data + cbDataReceived, nRead, image_buffer, nRead);
			//nRead = Receive(m_remote_image.data + cbDataReceived, 1460);

			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					// Trying to reconnect
					((CMapDataReceiverDlg*)m_parent)->DoLRFSocketConnect();
				}
				break;
			}

			//memcpy(m_remote_image.data + cbDataReceived, image_buffer, nRead);

			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);

		//ClearTCPBuffer();
		//ShowRemoteImage();
	}
	//ShowRemoteImage();

	CListBox* aListBox = (CListBox*)m_parent->GetDlgItem(IDC_SENSOR_DATA);
	aListBox->SetRedraw(false);
	aListBox->ResetContent();
	for (int i = 0; i < SIZE_LRF_DATA; i+=20) {
		char buffer[50];
		sprintf_s(buffer,50, "%3d : %2.4e", i, m_lrf_data[i]);
		aListBox->AddString(CString(buffer));
	}
	aListBox->SetRedraw(true);

	CSocket::OnReceive(nErrorCode);
}

BOOL CRSocket::OnMessagePending() {
	

	return 0;
}


void CRSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}
