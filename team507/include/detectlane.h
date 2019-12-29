#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <vector>
#include <math.h>
#include <algorithm>
#define w 320
#define h 240
using namespace std;
using namespace cv;
#define PI 3.14f
#define lost 60
class DetectLane
{
	public:
		
		DetectLane();
		~DetectLane();
		static int BIRDVIEW_WIDTH;
		static int BIRDVIEW_HEIGHT;
		vector<Point> tmpL,tmpR;
		
		Point centerP;
		Point centerPoint(const Mat &src);
		void update(const Mat &src);
		Point getPoint();
	private:
		Mat preProcess(const Mat &src);
		Mat laneInShadow(const Mat &src);
		Mat detect(const Mat &src);
		Mat birdViewTranform(const Mat &src);
		Mat View(const Mat &src);
		Mat cutROI(const Mat &src);
		Mat morphological(const Mat &img);
		vector<Vec4i> fillLane(Mat &src);
		Mat left_right_lanes(const Mat &src, vector<Vec4i> lines,Mat &dst1, Mat &dst2,bool &isL, bool &isR);
		Point calCenterPoint2(const Mat &src, const Mat &left, const Mat &right,const bool &isL, const bool &isR);
		Mat laneCanny(const Mat &src);

		

		int prevxTam = 0, prevLeft = 0, prevRight = 0;
		int skyLine = 85;

	//laneshadow
		int minShadow[3] = {63, 13, 60};
		int maxShadow[3] = {120, 100, 255};
	//lane
		int minThreshold[3] = {0, 0, 155};
		int maxThreshold[3] = {133, 40, 255};
		
		
		
};

#endif
