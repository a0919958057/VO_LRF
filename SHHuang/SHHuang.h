
// SHHuang.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"		// �D�n�Ÿ�


// CSHHuangApp:
// �аѾ\��@�����O�� SHHuang.cpp
//

class SHHuangApp : public CWinAppEx
{
public:
	SHHuangApp();

// �мg
	public:
	virtual BOOL InitInstance();

// �{���X��@

	DECLARE_MESSAGE_MAP()
};

extern SHHuangApp theApp;
