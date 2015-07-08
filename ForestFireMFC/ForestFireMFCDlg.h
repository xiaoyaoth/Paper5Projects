
// ForestFireMFCDlg.h : header file
//

#pragma once


// CForestFireMFCDlg dialog
class CForestFireMFCDlg : public CDialogEx
{
// Construction
public:
	CForestFireMFCDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_FORESTFIREMFC_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
	void myDraw();
	CDC _memDC;
	CBitmap _memBitmap;
	CDC *pDC;
	int screenWidth;
	int screenHeight;
	int fps;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
};
