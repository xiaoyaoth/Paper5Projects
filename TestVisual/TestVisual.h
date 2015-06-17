
// TestVisual.h : main header file for the TestVisual application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CTestVisualApp:
// See TestVisual.cpp for the implementation of this class
//

class CTestVisualApp : public CWinApp
{
public:
	CTestVisualApp();


// Overrides
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// Implementation

public:
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CTestVisualApp theApp;
