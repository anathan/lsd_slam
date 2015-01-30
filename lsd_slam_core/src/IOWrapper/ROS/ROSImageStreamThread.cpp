/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSImageStreamThread.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <boost/thread.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "util/settings.h"

#include <iostream>
#include <fstream>
#include <iostream>

#define WIN32_LEAN_AND_MEAN 
#define NOMINMAX
#include <windows.h>


namespace lsd_slam
{
	using namespace cv;

	HANDLE cameraHandle;
	cv::Mat cameraRaw;
	cv::Mat viewShared;
	cv::Mat camMat;
	cv::Mat distCoeff;

	void CamThread(ROSImageStreamThread*);

	DWORD WINAPI CamThreadWrap(LPVOID t)
	{
		ROSImageStreamThread* thread = (ROSImageStreamThread*)t;
		CamThread(thread);
		return 0;
	}

	void CamThread(ROSImageStreamThread* thread)
	{
		cv::VideoCapture capture("rtsp://192.168.0.123:554/live1.sdp");
		cv::Mat view0;
		boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();
		while (1) {
			if (capture.isOpened())
			{
				capture >> view0;
				TimestampedMat bufferItem;
				bufferItem.timestamp = Timestamp((boost::posix_time::second_clock::local_time()-start).total_seconds());
				bufferItem.data = view0.clone();
				thread->getBuffer()->pushBack(bufferItem);
			}
		}
	}






ROSImageStreamThread::ROSImageStreamThread()
{
	// subscribe
	/*vid_channel = nh_.resolveName("image");
	vid_sub        = nh_.subscribe(vid_channel,1, &ROSImageStreamThread::vidCb, this); */

	
	clock_t prevTimestamp = 0;
	cameraHandle = CreateThread(NULL, 0, CamThreadWrap, this, 0, NULL);

	// wait for cam calib
	width_ = height_ = 0;

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;
	lastSEQ = 0;

	haveCalib = false;
}

ROSImageStreamThread::~ROSImageStreamThread()
{
	delete imageBuffer;
}

void ROSImageStreamThread::setCalibration(std::string filename)
{
	std::cout << "Loading calibration file " << filename << std::endl;
	FileStorage fs(filename, FileStorage::READ);
	String s;
	int imageWidth;
	int imageHeight;

	fs["calibration_time"] >> s;
	fs["camera_matrix"] >> camMat;
	fs["distortion_coefficients"] >> distCoeff;
	fs["image_width"] >> imageWidth;
	fs["image_height"] >> imageHeight;

	std::cout << " Loaded camera calibration from " << s << " With Matrix " << camMat << " and distortion coeffs " << distCoeff << std::endl;
	for (int i = 0; i < 9; i++)
		std::cout << "index: " << i << " --> " << camMat.at<double>(i) << std::endl;

	//element 0 is fx
	// 2 ->  cx
	// 4- > fy
	// 5-> cy

	fx_ = camMat.at<double>(0);
	fy_ = camMat.at<double>(4);
	cx_ = camMat.at<double>(2);
	cy_ = camMat.at<double>(5);

	width_ = imageWidth;
	height_ = imageHeight;
	haveCalib = true;
}

void ROSImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSImageStreamThread::operator()()
{
	exit(0);
}

//
//void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img)
//{
//	if(!haveCalib) return;
//
//	//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//
//	if(img->header.seq < (unsigned int)lastSEQ)
//	{
//		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
//		lastSEQ = 0;
//		return;
//	}
//	lastSEQ = img->header.seq;
//
//	TimestampedMat bufferItem;
//	if(img->header.stamp.toSec() != 0)
//		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
//	else
//		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());
//
//	if(undistorter != 0)
//	{
//		assert(undistorter->isValid());
//		undistorter->undistort(cv_ptr->image,bufferItem.data);
//	}
//	else
//	{
//		bufferItem.data = cv_ptr->image;
//	}
//
//	imageBuffer->pushBack(bufferItem);
//}
//
//void ROSImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
//{
//	if(!haveCalib)
//	{
//		fx_ = info->P[0];
//		fy_ = info->P[5];
//		cx_ = info->P[2];
//		cy_ = info->P[6];
//
//		if(fx_ == 0 || fy_==0)
//		{
//			printf("camera calib from P seems wrong, trying calib from K\n");
//			fx_ = info->K[0];
//			fy_ = info->K[4];
//			cx_ = info->K[2];
//			cy_ = info->K[5];
//		}
//
//		width_ = info->width;
//		height_ = info->height;
//
//		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
//	}
//}

}
