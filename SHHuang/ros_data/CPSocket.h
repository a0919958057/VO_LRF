#pragma once
#include "afxsock.h"

#define SIZE_POSE_DATA 3
class CRSocket :
	public CSocket {
public:
	CRSocket();
	virtual ~CRSocket();
	virtual void OnReceive(int nErrorCode);
	virtual BOOL OnMessagePending();
	double m_pose_data[SIZE_POSE_DATA];
	static double Laser_pos[SIZE_POSE_DATA];
	void registerParent(CWnd* _parent);

	CWnd* m_parent;
};

