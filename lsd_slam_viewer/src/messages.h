#ifndef KEYFRAMEMSG_H_
#define KEYFRAMEMSG_H_

#include <string>
#include <vector>
#include <map>
#include <ostream>

class KeyframeMsg;
class KeyframeGraphMsg;

typedef const KeyframeMsg* KeyframeMsgConstPtr;
typedef const KeyframeGraphMsg* KeyframeGraphMsgConstPtr;

class KeyframeMsg {
public:
	bool isKeyframe;
	int id;
	double time;
	float fx, fy, cx, cy, width, height;
	float camToWorld[7];
	unsigned char* pointcloud;

};

class KeyframeGraphMsg {
public:
	int numConstraints;
	int numFrames;
	unsigned char*  frameData;
	unsigned char*  constraintsData;

};




#endif