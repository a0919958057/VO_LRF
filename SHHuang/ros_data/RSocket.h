#pragma once
#include "afxsock.h"

#define SIZE_LRF_DATA 667
class CRSocket :
	public CSocket {
public:
	CRSocket();
	virtual ~CRSocket();
	virtual void OnReceive(int nErrorCode);
	virtual BOOL OnMessagePending();
	float m_lrf_data[SIZE_LRF_DATA];
	void registerParent(CWnd* _parent);

	CWnd* m_parent;
};

