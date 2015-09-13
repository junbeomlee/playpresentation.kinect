#pragma once

#include "stdafx.h"
#include "BodyIndexCV.h"



CBodyIndexCV::CBodyIndexCV(IKinectSensor* pKinectSensor) :
m_pKinectSensor(NULL),
m_pBodyIndexFrameReader(NULL)
{
	m_pKinectSensor = pKinectSensor;
	initBodyIndexReader();
	m_bodyIndexMat = Mat::zeros(cBodyIndexHeight, cBodyIndexWidth, CV_16UC4);
}

CBodyIndexCV::CBodyIndexCV() :
	m_pKinectSensor(NULL),
	m_pBodyIndexFrameReader(NULL)
{
	m_bodyIndexMat = Mat::zeros(cBodyIndexHeight, cBodyIndexWidth, CV_16UC4);
}

CBodyIndexCV::~CBodyIndexCV()
{
	m_bodyIndexMat.release();
	SafeRelease(m_pBodyIndexFrameReader);
}

HRESULT CBodyIndexCV::initBodyIndexReader()
{
	HRESULT hr;
	IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;

	hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);

	if (SUCCEEDED(hr))
	{
		hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
	}

	SafeRelease(pBodyIndexFrameSource);

	if (!m_pKinectSensor || SUCCEEDED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

void CBodyIndexCV::UpdateKinectBodyIndexFrame(Mat depthMat)
{
	if (!m_pBodyIndexFrameReader)
	{
		return;
	}

	IBodyIndexFrame* pBodyIndexFrame = NULL;

	HRESULT hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	cvtColor(depthMat, m_bodyIndexMat, CV_GRAY2BGR);

	if (SUCCEEDED(hr))
	{
		ProcessBodyIndex(pBodyIndexFrame);
	}

	SafeRelease(pBodyIndexFrame);
}

void CBodyIndexCV::ProcessBodyIndex(IBodyIndexFrame* pBodyIndexFrame)
{
	HRESULT hr;
	unsigned int bufferSize = 0;
	BYTE* buffer;
	cv::Vec3w color[BODY_COUNT];
	color[0] = cv::Vec3w(pow(2, 16) - 1, 0, 0);
	color[1] = cv::Vec3w(0, pow(2, 16) - 1, 0);
	color[2] = cv::Vec3w(0, 0, pow(2, 16) - 1);
	color[3] = cv::Vec3w(pow(2, 16) - 1, pow(2, 16) - 1, 0);
	color[4] = cv::Vec3w(pow(2, 16) - 1, 0, pow(2, 16) - 1);
	color[5] = cv::Vec3w(0, pow(2, 16) - 1, pow(2, 16) - 1);

	bufferSize = cBodyIndexHeight * cBodyIndexWidth * sizeof(unsigned short);
	free(m_pBodyIndexBuffer);
	m_pBodyIndexBuffer = (BYTE*)malloc(bufferSize);
	hr = pBodyIndexFrame->AccessUnderlyingBuffer(&bufferSize, &buffer);
	
	memcpy(m_pBodyIndexBuffer, buffer, bufferSize);
	if (SUCCEEDED(hr)){
		if (SUCCEEDED(hr)){
			for (int y = 0; y < cBodyIndexHeight; y++){
				for (int x = 0; x < cBodyIndexWidth; x++){
					unsigned int index = y * cBodyIndexWidth + x;
					if (buffer[index] != 0xff){
						m_bodyIndexMat.at<cv::Vec3w>(y, x) = color[buffer[index]];
					}
					else{
						m_bodyIndexMat.at<cv::Vec3w>(y, x) = cv::Vec3w(0, 0, 0);
					}
				}
			}
		}
	}
}