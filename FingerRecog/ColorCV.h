#pragma once

#include "stdafx.h"

class CColorCV {
	static const int cColorWidth = 1920;
	static const int cColorHeight = 1080;

private:
	IKinectSensor*			m_pKinectSensor;
	IColorFrameReader*		m_pColorFrameReader;
	Mat						m_colorMat;

public:
	CColorCV();
	CColorCV(IKinectSensor* pKinectSensor);
	~CColorCV();

	HRESULT					initColorReader();
	void					UpdateKinectColorFrame();
	void					ProcessColor(IColorFrame* pColorFrame);
	Mat						GetColorMat() { return m_colorMat; };
};

