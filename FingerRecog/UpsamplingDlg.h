#pragma once


// CUpsamplingDlg ��ȭ �����Դϴ�.

class CUpsamplingDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CUpsamplingDlg)

public:
	CUpsamplingDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CUpsamplingDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_UP_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};
