#ifndef PCAMERA_H
#define PCAMERA_H

#pragma once
#include<iostream>
using namespace std;

#include "FlyCapture2.h"
using namespace FlyCapture2;


class PCamera
{
public:
    PCamera();

	FlyCapture2::Error m_Error;

    Camera m_Cam;

    TriggerMode m_TriggerMode;

public:
    bool ConnectToCamera();
	//连接相机的重载函数（可以传递参数）
	bool ConnectToCamera(unsigned int camFlag);
    bool ConnectToCamera2();
    bool DisconnectCamera();
	void PrintError(FlyCapture2::Error error);
    void PrintCameraInfo( CameraInfo* pCamInfo );
    bool FireSoftwareTrigger( Camera* pCam );
    void GrabAPicture(Image& pgImage);
    bool SetExposureTime(unsigned char ms);
	bool SetExposureTime(float ms);
    //设置视频模式
    bool setVideo();
};

#endif // PCAMERA_H
