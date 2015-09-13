
// FingerRecogDlg.h : ��� ����
//

#pragma once


// CFingerRecogDlg ��ȭ ����
class CFingerRecogDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CFingerRecogDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_FINGERRECOG_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCalibButton();
	afx_msg void OnBnClickedHandButton();
	afx_msg void OnBnClickedUpButton();
	afx_msg void OnBnClickedFingerButton();
};
