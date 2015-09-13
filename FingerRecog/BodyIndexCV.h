#pragma once

#include "stdafx.h"

class CBodyIndexCV {
	static const int cBodyIndexWidth = 512;
	static const int cBodyIndexHeight = 424;

private:
	IKinectSensor*				m_pKinectSensor;
	IBodyIndexFrameReader*		m_pBodyIndexFrameReader;
	Mat							m_bodyIndexMat;
	BYTE*						m_pBodyIndexBuffer;
public:
	CBodyIndexCV();
	CBodyIndexCV(IKinectSensor* pKinectSensor);
	~CBodyIndexCV();

	HRESULT						initBodyIndexReader();
	void						UpdateKinectBodyIndexFrame(Mat depthMat);
	void						ProcessBodyIndex(IBodyIndexFrame* pBodyIndexFrame);
	BYTE*						GetBodyIndexBuffer() { return m_pBodyIndexBuffer; };
	Mat							GetBodyIndexMat() { return m_bodyIndexMat; };
};

