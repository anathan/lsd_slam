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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/


//#include "ros/ros.h"
#include "boost/thread.hpp"
#include "settings.h"
#include <iostream>
#include "PointCloudViewer.h"

#include <qapplication.h>


#include "boost/foreach.hpp"


PointCloudViewer* viewer = 0;

/*

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

	if(msg->time > lastFrameTime) return;

	if(viewer != 0)
		viewer->addFrameMsg(msg);
}

void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(viewer != 0)
		viewer->addGraphMsg(msg);
}


*/

int main( int argc, char** argv )
{


	printf("Started QApplication thread\n");
	// Read command lines arguments.
	QApplication application(argc,argv);

	// Instantiate the viewer.
	viewer = new PointCloudViewer();


	#if QT_VERSION < 0x040000
		// Set the viewer as the application main widget.
		application.setMainWidget(viewer);
	#else
		viewer->setWindowTitle("PointCloud Viewer");
	#endif

	// Make the viewer window visible on screen.
	viewer->show();


	application.exec();

	std::cout<<"Shutting down... \n";
	

}
