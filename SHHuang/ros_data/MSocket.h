#pragma once
#include "afxsock.h"
#include "../server/include/MapDataStruct.h"
#include <boost\shared_ptr.hpp>

#include <vector>


// Show the socket receiving status
#define _SHOW_SOCKET_DEBUG
using namespace std;

typedef boost::shared_ptr<MapDataPart> MapDataPartPtr;
typedef vector<MapDataPartPtr> MapDataPartArray;

class CMSocket :
	public CSocket {
public:
	CMSocket();
	virtual ~CMSocket();
	virtual void OnReceive(int nErrorCode);
	virtual BOOL OnMessagePending();
	void registerParent(CWnd* _parent);
	CWnd* m_parent;
	struct MapData* m_map_data;
	MapDataPartArray m_map_parts;
	void showData();
private:
	void assembleData();
};

