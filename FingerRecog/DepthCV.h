#pragma once

#include "stdafx.h"

class CDepthCV {
	static const int cDepthWidth = 512;
	static const int cDepthHeight = 424;

private:
	IKinectSensor*			m_pKinectSensor;
	IDepthFrameReader*		m_pDepthFrameReader;
	Mat						m_depthMat;

public:
	CDepthCV();
	CDepthCV(IKinectSensor* pKinectSensor);
	~CDepthCV();

	HRESULT					initDepthReader();
	void					UpdateKinectDepthFrame();
	void					ProcessDepth(IDepthFrame* pDepthFrame);
	Mat						GetDepthMat() { return m_depthMat; };
};

