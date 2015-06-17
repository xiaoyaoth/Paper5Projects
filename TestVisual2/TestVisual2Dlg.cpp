
// TestVisual2Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "TestVisual2.h"
#include "TestVisual2Dlg.h"
#include "afxdialogex.h"

#include "SocialForce.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

SocialForceSimApp cloneApp;
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
	//	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
	//	ON_WM_LBUTTONDBLCLK()
END_MESSAGE_MAP()


// CTestVisual2Dlg dialog



CTestVisual2Dlg::CTestVisual2Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CTestVisual2Dlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CTestVisual2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CTestVisual2Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CTestVisual2Dlg::OnBnClickedOk)
	ON_WM_TIMER()
	ON_WM_ERASEBKGND()
	ON_WM_LBUTTONDBLCLK()
END_MESSAGE_MAP()


// CTestVisual2Dlg message handlers

BOOL CTestVisual2Dlg::OnInitDialog()
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
	screenWidth = 400;
	screenHeight = 400;
	redPen = new CPen(PS_SOLID, 2, RGB(255, 0, 0));
	bluePen = new CPen(PS_SOLID, 2, RGB(0, 0, 255));
	greenPen = new CPen(PS_SOLID, 2, RGB(0, 255, 0));
	redBrush = new CBrush(RGB(255, 0, 0));
	blueBrush = new CBrush(RGB(0, 0, 255));
	greenBrush = new CBrush(RGB(0, 255, 0));
	MoveWindow(0, 0, screenWidth, screenHeight, true);
	SetTimer(1, 10, NULL);
	cloneApp.initSimClone();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CTestVisual2Dlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CTestVisual2Dlg::OnPaint()
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
HCURSOR CTestVisual2Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CTestVisual2Dlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here

	/*
	CRect rect;
	GetClientRect(&rect);
	rect.bottom = 100;
	rect.top = 20;
	rect.left = 20;
	rect.right = 100;
	CClientDC dc(this);
	CBrush *blk_brush = new CBrush(RGB(0, 0, 0));
	CPen *pen = new CPen(PS_SOLID, 2, RGB(0, 0, 255));
	dc.SelectObject(pen);
	dc.SelectObject(blk_brush);
	dc.Rectangle(rect);

	dc.Ellipse(100,100,120,150);
	*/

}


void CTestVisual2Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	cloneApp.stepApp();
	myDraw();

	if (nIDEvent == 1){

	}
	CDialogEx::OnTimer(nIDEvent);
}


BOOL CTestVisual2Dlg::OnEraseBkgnd(CDC* pDC)
{
	// TODO: Add your message handler code here and/or call default

	//return CDialogEx::OnEraseBkgnd(pDC);
	return FALSE;
}


void CTestVisual2Dlg::myDraw()
{
	CClientDC dc(this);
	_memDC.CreateCompatibleDC(NULL);
	_memBitmap.CreateCompatibleBitmap(pDC, screenWidth, screenHeight);
	CBitmap *pOldBitmap = _memDC.SelectObject(&_memBitmap);
	_memDC.SetBkColor(RGB(0, 0, 0));
	_memDC.SetBkMode(TRANSPARENT);
	CRect rect0(0, 0, screenWidth, screenHeight + 50);
	_memDC.Rectangle(rect0);

	_memDC.SelectStockObject(NULL_BRUSH);
	int paintId = cloneApp.paintId;
	SocialForceClone *c = cloneApp.cAll[paintId];

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
	_memDC.SelectObject(greenPen);
	_memDC.SelectObject(greenBrush);
	for (int i = 0; i < NUM_PARAM; i++) {
		double2 c1(32, 32), c2(32, 96), c3(96, 32), c4(96, 96);
		double x1, y1, x2, y2;
		if (c->pv[0] == 1)	{
			x1 = (c1.x - 3) / ENV_DIM * screenWidth;
			y1 = (c1.y - 3) / ENV_DIM * screenHeight;
			x2 = (c1.x + 3) / ENV_DIM * screenWidth;
			y2 = (c1.y + 3) / ENV_DIM * screenHeight;
			_memDC.Ellipse(x1, y1, x2, y2);
		}
		if (c->pv[1] == 1)	{
			x1 = (c2.x - 3) / ENV_DIM * screenWidth;
			y1 = (c2.y - 3) / ENV_DIM * screenHeight;
			x2 = (c2.x + 3) / ENV_DIM * screenWidth;
			y2 = (c2.y + 3) / ENV_DIM * screenHeight;
			_memDC.Ellipse(x1, y1, x2, y2);
		}
		if (c->pv[2] == 1)	{
			x1 = (c3.x - 3) / ENV_DIM * screenWidth;
			y1 = (c3.y - 3) / ENV_DIM * screenHeight;
			x2 = (c3.x + 3) / ENV_DIM * screenWidth;
			y2 = (c3.y + 3) / ENV_DIM * screenHeight;
			_memDC.Ellipse(x1, y1, x2, y2);
		}
		if (c->pv[3] == 1)	{
			x1 = (c4.x - 3) / ENV_DIM * screenWidth;
			y1 = (c4.y - 3) / ENV_DIM * screenHeight;
			x2 = (c4.x + 3) / ENV_DIM * screenWidth;
			y2 = (c4.y + 3) / ENV_DIM * screenHeight;
			_memDC.Ellipse(x1, y1, x2, y2);
		}
	}

	//for (int i = 0; i < 100; i++) {
	//	double x = rand() % screenWidth;
	//	double y = rand() % screenHeight;
	//	_memDC.Ellipse(x - 3, y - 3, x + 3, y + 3);
	//}

	pDC->BitBlt(0, 0, screenWidth, screenHeight + 50, &_memDC, 0, 0, SRCCOPY);
	_memBitmap.DeleteObject();
	_memDC.DeleteDC();

	UpdateData(false);
}


void CTestVisual2Dlg::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	cloneApp.paintId++;
	CDialogEx::OnLButtonDblClk(nFlags, point);
}
