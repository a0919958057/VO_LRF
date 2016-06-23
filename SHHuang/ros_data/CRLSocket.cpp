#include "stdafx.h"
#include "CRLSocket.h"
#include "MapDataReceiverDlg.h"
#include "resource.h"


CCRLSocket::CCRLSocket() {
	int buffer_size = 1024 * 1024 * 16;
	SetSockOpt(SO_RCVBUF, &buffer_size, sizeof(int));
}


CCRLSocket::~CCRLSocket() {
}

void CCRLSocket::OnReceive(int nErrorCode) {
	static int counter(0);
	counter++;

	if (0 == nErrorCode) {

		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(sizeof(ControlMsg)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive(&m_cmd_msg + cbDataReceived, cbLeft);


#ifdef _SHOW_SOCKET_DEBUG

			/****** Print Socket status inorder to debug ****/
			CString msg;
			msg.Format(_T("Control Msg Recivced: %d"), counter);
			m_parent->SetDlgItemTextW(IDC_RECE_STATUS2, msg);
			m_parent->UpdateData(false);
			/**********************************************/

			if (cbLeft < 0) AfxMessageBox(_T("WTF occurred"));
#endif // _SHOW_SOCKET_DEBUG


			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					// Trying to reconnect
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



	showData();

	CSocket::OnReceive(nErrorCode);
}

BOOL CCRLSocket::OnMessagePending() {


	return 0;
}

void CCRLSocket::showData() {
	// ***********************************
	// Show the Report data to User
	// ***********************************
	CListBox* alist = (CListBox*)m_parent->GetDlgItem(IDC_MAP_LIST);
	alist->ResetContent();
	CString msg;
	for (int i = 0; i < REPORT_DATA_SIZE; i++) {

		msg.Format(_T("%d.Map:%d"), i, m_cmd_msg.data[i]);
		alist->AddString(msg);
	}
	

}

void CCRLSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}

