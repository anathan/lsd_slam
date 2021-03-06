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

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"


#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"

using namespace lsd_slam;
int main( int argc, char** argv )
{
  


	InputImageStream* inputStream = new ROSImageStreamThread();

	std::string calibFile;
	inputStream->setCalibration("c:\\out_camera_data_dlink_12-27.yml");
	//inputStream->run();

	Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(inputStream->width(), inputStream->height());
	LiveSLAMWrapper slamNode(inputStream, outputWrapper);
	slamNode.Loop();



	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	return 0;
}
