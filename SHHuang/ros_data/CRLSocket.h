#pragma once
#include "afxsock.h"
#include "../server/include/ControlDataStruct.h"

#include <boost\shared_ptr.hpp>

#include <vector>

class CCRLSocket :
	public CSocket {
public:
	CCRLSocket();
	~CCRLSocket();
	virtual void OnReceive(int nErrorCode);
	virtual BOOL OnMessagePending();
	void registerParent(CWnd* _parent);
	CWnd* m_parent;
	struct ControlMsg m_cmd_msg;
	void showData();
};

