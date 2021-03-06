
// ForestFireMFCDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ForestFireMFC.h"
#include "ForestFireMFCDlg.h"
#include "afxdialogex.h"

#include "ForestFire.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

ForestFireSimApp cloneApp;
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
public:
//	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
//	ON_WM_TIMER()
END_MESSAGE_MAP()


// CForestFireMFCDlg dialog



CForestFireMFCDlg::CForestFireMFCDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CForestFireMFCDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CForestFireMFCDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CForestFireMFCDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_LBUTTONDBLCLK()
END_MESSAGE_MAP()


// CForestFireMFCDlg message handlers

BOOL CForestFireMFCDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	pDC = this->GetDC();
	fps = 100;
	SetTimer(1, 1000 / fps, NULL); 
	cloneApp.init();
	cellSize = 3;
	screenWidth = cloneApp.ncols * cellSize;
	screenHeight = cloneApp.nrows * cellSize;
	MoveWindow(0, 0, screenWidth + 18, screenHeight + 40, true);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CForestFireMFCDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CForestFireMFCDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CForestFireMFCDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CForestFireMFCDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	cloneApp.step();
	myDraw();

	CDialogEx::OnTimer(nIDEvent);
}

void CForestFireMFCDlg::myDraw() {
	CDC _memDC;
	_memDC.CreateCompatibleDC(NULL);
	_memBitmap.CreateCompatibleBitmap(pDC, screenWidth, screenHeight);
	CBitmap *pOldBitmap = _memDC.SelectObject(&_memBitmap);
	_memDC.SetBkColor(RGB(0, 0, 0));
	_memDC.SetBkMode(TRANSPARENT);
	CRect rect0(0, 0, screenWidth, screenHeight + 50);
	_memDC.Rectangle(rect0);

	_memDC.SelectStockObject(NULL_BRUSH);
	for (int i = 0; i < cloneApp.nrows; i++) {
		for (int j = 0; j < cloneApp.ncols; j++) {
			int val = cloneApp.paintMap[i][j];
			val = val == -9999 ? 0 : 50 + val * 200 / cloneApp.paintMax;
			//val = rand() % 255;
			CPen p(PS_SOLID, 0, RGB(val, val, val));
			CBrush b(RGB(val, val, val));
			_memDC.SelectObject(p);
			_memDC.SelectObject(b);
			_memDC.Rectangle(j * cellSize, i * cellSize, (j + 1) * cellSize, (i + 1) * cellSize);
		}
	}

	// before draw
	/*
	_memDC.SelectStockObject(NULL_BRUSH);
	int paintId = cloneApp.paintId;
	SocialForceClone *c = cloneApp.cAll[paintId];

	// draw title
	WCHAR title[100];
	swprintf_s(title, 100, L"parent: %d, clone: %d, numElem: %d, step: %d", c->parentCloneid, c->cloneid, c->ap->numElem, cloneApp.stepCount);
	this->SetWindowText((LPCTSTR)title);

	// draw passive clone area 
	for (int i = 0; i < NUM_CELL; i++) {
		for (int j = 0; j < NUM_CELL; j++) {
			if (cloneApp.cAll[paintId]->takenMap[i * NUM_CELL + j]) {
				CPen p(PS_SOLID, 0, RGB(240, 240, 240));
				CBrush b(RGB(240, 240, 240));
				_memDC.SelectObject(p);
				_memDC.SelectObject(b);
				int wscale = screenWidth / NUM_CELL;
				int hscale = screenHeight / NUM_CELL;
				_memDC.Rectangle(i * wscale, j * hscale, (i + 1) * wscale, (j + 1) * hscale);
			}
		}
	}

	// draw grid
	int numLine = NUM_CELL;
	for (int i = 0; i < numLine; i++) {
		char rgb = 100;
		CPen p(PS_DOT, 0, RGB(rgb, rgb, rgb));
		_memDC.SelectObject(p);
		_memDC.MoveTo(i * screenWidth / NUM_CELL, 0);
		_memDC.LineTo(i * screenWidth / NUM_CELL, screenHeight);
		_memDC.MoveTo(0, i * screenHeight / NUM_CELL);
		_memDC.LineTo(screenWidth, i * screenHeight / NUM_CELL);
	}


	// draw wall
	for (int i = 0; i < NUM_WALLS; i++) {
		CPen p(PS_SOLID, 5, RGB(0, 0, 0));
		_memDC.SelectObject(p);
		double x = c->walls[i].sx / ENV_DIM * screenWidth;
		double y = c->walls[i].sy / ENV_DIM * screenHeight;
		_memDC.MoveTo(x, y);
		x = c->walls[i].ex / ENV_DIM * screenWidth;
		y = c->walls[i].ey / ENV_DIM * screenHeight;
		_memDC.LineTo(x, y);
	}

	// draw agent
	for (int i = 0; i < NUM_CAP; i++) {
		SocialForceAgent *a = c->context[i];
		CPen p(PS_SOLID, 2, RGB(a->color.r, a->color.g, a->color.b));
		CBrush b(RGB(a->color.r, a->color.g, a->color.b));
		_memDC.SelectObject(p);
		_memDC.SelectObject(b);
		double x = c->context[i]->data.loc.x / ENV_DIM * screenWidth;
		double y = c->context[i]->data.loc.y / ENV_DIM * screenHeight;
		_memDC.Ellipse(x - 3, y - 3, x + 3, y + 3);
	}
	*/

	// after draw
	pDC->BitBlt(0, 0, screenWidth, screenHeight, &_memDC, 0, 0, SRCCOPY);
	_memBitmap.DeleteObject();
	_memDC.DeleteDC();

	UpdateData(false);
}


void CForestFireMFCDlg::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	cloneApp.paintLayer = (cloneApp.paintLayer + 1) % 6;
	WCHAR title[128];
	switch (cloneApp.paintLayer) {
	case 0:
		cloneApp.paintMap = cloneApp.elevMap;
		cloneApp.paintMax = cloneApp.maxElev;
		swprintf_s(title, 128, L"elevation map");
		this->SetWindowText((LPCTSTR)title);
		break;
	case 1:
		cloneApp.paintMap = cloneApp.slopeMap;
		cloneApp.paintMax = cloneApp.maxSlope;
		swprintf_s(title, 128, L"slope map");
		this->SetWindowText((LPCTSTR)title);
		break;
	case 2:
		cloneApp.paintMap = cloneApp.aspectMap;
		cloneApp.paintMax = cloneApp.maxAspect;
		swprintf_s(title, 128, L"aspect map");
		this->SetWindowText((LPCTSTR)title);
		break;
	case 3:
		cloneApp.paintMap = cloneApp.fuelMap;
		cloneApp.paintMax = cloneApp.maxFuel;
		swprintf_s(title, 128, L"fuel map");
		this->SetWindowText((LPCTSTR)title);
		break;
	case 4:
		cloneApp.paintMap = cloneApp.canopyMap;
		cloneApp.paintMax = cloneApp.maxCanopy;
		swprintf_s(title, 128, L"canopy map");
		this->SetWindowText((LPCTSTR)title);
		break;
	case 5:
		cloneApp.paintMap = cloneApp.heightMap;
		cloneApp.paintMax = cloneApp.maxHeight;
		swprintf_s(title, 128, L"height map");
		this->SetWindowText((LPCTSTR)title);
	}
	CDialogEx::OnLButtonDblClk(nFlags, point);
}
