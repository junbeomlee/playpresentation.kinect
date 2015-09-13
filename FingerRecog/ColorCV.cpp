#pragma once

#include "stdafx.h"
#include "ColorCV.h"


CColorCV::CColorCV() :
	m_pKinectSensor(NULL),
	m_pColorFrameReader(NULL)
{
	m_colorMat = Mat(cColorHeight, cColorWidth, CV_8UC4);
}

CColorCV::CColorCV(IKinectSensor* pKinectSensor) :
m_pKinectSensor(NULL),
m_pColorFrameReader(NULL)
{
	m_pKinectSensor = pKinectSensor;
	initColorReader();
	m_colorMat = Mat(cColorHeight, cColorWidth, CV_8UC4);
}


CColorCV::~CColorCV()
{
	m_colorMat.release();
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pKinectSensor);
}

HRESULT CColorCV::initColorReader()
{
	HRESULT hr;
	IColorFrameSource* pColorFrameSource = NULL;

	hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);

	if (SUCCEEDED(hr))
	{
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}

	SafeRelease(pColorFrameSource);

	if (!m_pKinectSensor || SUCCEEDED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

void CColorCV::UpdateKinectColorFrame()
{
	if (!m_pColorFrameReader)
	{
		return;
	}

	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		ProcessColor(pColorFrame);
	}

	SafeRelease(pColorFrame);
}

void CColorCV::ProcessColor(IColorFrame* pColorFrame)
{
	HRESULT hr; 
	ColorImageFormat imageFormat;
	UINT nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
	RGBQUAD *pColorBuffer = NULL;

	hr = pColorFrame->get_RawColorImageFormat(&imageFormat);

	if (SUCCEEDED(hr))
	{
		if (imageFormat == ColorImageFormat_Bgra)
		{
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
		}
		else
		{
			hr = E_FAIL;
		}
	}

	hr = pColorFrame->CopyConvertedFrameDataToArray(m_colorMat.dataend - m_colorMat.datastart, m_colorMat.data, ColorImageFormat_Bgra);
}