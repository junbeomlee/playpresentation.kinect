#pragma once

#include "stdafx.h"
#include "DepthCV.h"


CDepthCV::CDepthCV() :
	m_pKinectSensor(NULL),
	m_pDepthFrameReader(NULL)
{
	m_depthMat = Mat(cDepthHeight, cDepthWidth, CV_16UC1);
}

CDepthCV::CDepthCV(IKinectSensor* pKinectSensor) :
m_pKinectSensor(NULL),
m_pDepthFrameReader(NULL)
{
	m_pKinectSensor = pKinectSensor;
	initDepthReader();
	m_depthMat = Mat(cDepthHeight, cDepthWidth, CV_16UC1);
}

CDepthCV::~CDepthCV()
{
	//m_depthMat.release();

	SafeRelease(m_pDepthFrameReader);
}

HRESULT CDepthCV::initDepthReader()
{
	HRESULT hr;
	IDepthFrameSource* pDepthFrameSource = NULL;

	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}

	SafeRelease(pDepthFrameSource);

	if (!m_pKinectSensor || SUCCEEDED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

void CDepthCV::UpdateKinectDepthFrame()
{
	if (!m_pDepthFrameReader)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		ProcessDepth(pDepthFrame);
	}

	SafeRelease(pDepthFrame);
}

void CDepthCV::ProcessDepth(IDepthFrame* pDepthFrame)
{
	HRESULT hr;
	unsigned int bufferSize = 0;
	UINT16* buffer;
	bufferSize = cDepthHeight * cDepthWidth * sizeof(unsigned short);
	hr = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &buffer);
	bufferSize = cDepthHeight * cDepthWidth * sizeof(unsigned short);
	if (SUCCEEDED(hr))
	{
		memcpy(m_depthMat.data, buffer, bufferSize);
	}
}