#include "detectlane.h"
DetectLane::DetectLane(){ 
}
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
DetectLane::~DetectLane(){}
Mat DetectLane::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst, lane1, lane2;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minThreshold[0],minThreshold[1],minThreshold[2]),
                Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), lane1);
	inRange(imgHSV, Scalar(minShadow[0],minShadow[1],minShadow[2]),
			Scalar(maxShadow[0], maxShadow[1], maxShadow[2]), lane2);
	lane1 = detect(lane1);
	lane2 = detect(lane2);
	Mat temp = lane1 | lane2;
	dst = cutROI(temp);
	dst = detect(dst);
	imshow("lane", dst);
	return dst;
}
Mat DetectLane::cutROI(const Mat &src){
    Mat dst;
    int _h = src.rows, _w = src.cols;
    Mat mask = Mat::zeros(src.size(), src.type());

    Point pts2[4] = {
            Point(0, _h),
            Point(0, 80),
            Point(_w, 80),
            Point(_w, _h)
    };

    fillConvexPoly(mask, pts2, 4, Scalar(255));
    bitwise_and(src, mask, dst);

    return dst;
}
Mat DetectLane::View(const Mat &src)
{
	int width = src.size().width;
	int height = src.size().height;
	Mat dst = src.clone();
	Rect view(0, height/2-10, width, height/4-10);
	Mat ROI = dst(view);
	return ROI;
}

vector<Vec4i> DetectLane::fillLane(Mat &src)
{
	vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/90, 50,10,30);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
	}
	return lines;
}
Mat DetectLane::left_right_lanes(const Mat &src, vector<Vec4i> lines, Mat &dst1, Mat &dst2,bool &isL, bool &isR)
{
	isL = false;
	isR = false;
	Mat dst = src.clone();
	vector<Vec4i> left, right;
	left.clear();
	right.clear();
	for(size_t i = 0; i < lines.size(); i++)
	{
		Point p1, p2;
		p1.x = lines[i][0];
		p1.y = lines[i][1];
		p2.x = lines[i][2];
		p2.y = lines[i][3];

		double x = (double)(p2.x - p1.x);
		double y = (double)(p2.y - p1.y);
		int px = (p1.x + p2.x)/2;
		int py = (p1.y + p2.y)/2;
		if(p1.x != p2.x){
		float angle = atan2(y, x)*180/CV_PI;
		float slope = y/x;
		if(slope > 0)
			{
				if(angle > 20 && angle < 85 && y > 0 && (px>src.cols/2))
				{
					right.push_back(lines[i]);
					isR = true;
				}
			
			}
		else if(slope < 0)
			{
				if(angle < -20 && angle > -85 && y < 0 && (px<src.cols/2))
				{
					left.push_back(lines[i]);
					isL = true;
				}
			}
		}
	}
	for(size_t j = 0; j < right.size(); j++)
	{
		line(dst1, Point(right[j][0], right[j][1]), Point(right[j][2], right[j][3]), Scalar(255), 2);
		line(dst, Point(right[j][0], right[j][1]), Point(right[j][2], right[j][3]), Scalar(255), 2);
	}
	for(size_t j = 0; j < left.size(); j++)
	{
		line(dst2, Point(left[j][0], left[j][1]), Point(left[j][2], left[j][3]), Scalar(255), 2);
		line(dst, Point(left[j][0], left[j][1]), Point(left[j][2], left[j][3]), Scalar(255), 2);
	}
	return dst;
}
Point DetectLane::calCenterPoint2(const Mat &src, const Mat &left, const Mat &right,const bool &isL, const bool &isR)
{
	int xLeft = 0, yLeft = 0, xRight = 0, yRight = 0;
	int xTam = 0, yTam = 0;
	vector<Point> lanesLeft, lanesRight;
	lanesLeft.clear();
	lanesRight.clear();
	int wSrc = src.cols;
	int hSrc = src.rows;
	if(isR && isL)
	{
		for(int i = 0; i < hSrc; i++)
		{
			for(int j = wSrc/2; j >=0; j--)
			{
				if(src.at<uchar>(i,j) == 255)
				{
					lanesLeft.push_back(Point(j,i));
					break;
				}
			}
		}
		for(int i = 0; i < hSrc; i++)
		{
			for(int j = wSrc/2; j < wSrc; j++)
			{
				if(src.at<uchar>(i,j) == 255)
				{
					lanesRight.push_back(Point(j,i));
					break;
				}
			}
		}	

		int nLeft = lanesLeft.size(), nRight = lanesRight.size();
		for(int i = 0; i < nLeft; i++)
		{
			xLeft += lanesLeft[i].x;
			yLeft += lanesLeft[i].y;
		}
		for(int i = 0; i < nRight; i++)
		{
			xRight += lanesRight[i].x;
			yRight += lanesRight[i].y;
		}
		xTam = (xLeft/nLeft + xRight/nRight)/2;
		yTam = (yLeft/nLeft + yRight/nRight)/2;
		yTam += h/2; //h*3/4  --h: height of origin image
		return Point(xTam,yTam);
	}else if(isL && !isR)
	{
		for(int i = 0; i < hSrc; i++)
			for(int j = wSrc-1; j >=0; j--)
				if(left.at<uchar>(i,j) == 255)
					{
						lanesLeft.push_back(Point(j,i));
						break;
					}
		int nLeft = lanesLeft.size();
		for(int i = 0; i < nLeft; i++)
		{
			xLeft += lanesLeft[i].x;
			yLeft += lanesLeft[i].y;
		}
		xTam = (xLeft/nLeft);
		yTam = (yLeft/nLeft + 0) ;
		yTam += h/2;
		return Point(xTam,yTam);
	}else if(!isL && isR)
	{
		for(int i = 0; i < hSrc; i++)
			for(int j = 0; j < wSrc; j++)
				if(right.at<uchar>(i,j) == 255)
					{
						lanesRight.push_back(Point(j,i));
						break;
					}
		int nRight = lanesRight.size();
		for(int i = 0; i < nRight; i++)
		{
			xRight += lanesRight[i].x;
			yRight += lanesRight[i].y;
		}
		xTam = (xRight/nRight);
		yTam = (yRight/nRight + 0);
		yTam += h/2;
		return Point(xTam,yTam);
	}else
	{
		xTam = w/2;
		yTam = hSrc/2 + h/2;
		return Point(xTam,yTam);
	}

}
Mat DetectLane::detect(const Mat &bin)
{
	Mat dst = Mat::zeros(bin.size(), bin.type());
    vector< vector<Point> > contours;
    vector<Vec4i> hierachy;
    findContours(bin, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    vector<Rect> rects(0);

    if(contours.size() > 0)
    {
        for(int i = 0; i < contours.size(); i++)
        {
            Rect rc = boundingRect(contours[i]);
            if(rc.height >= 30 && rc.width >= 5)
            {
                drawContours(dst, contours, i, Scalar(255), -1);
            }
        }
    }
    return dst;
}
void DetectLane::update(const Mat &src)
{
	Mat left,right;
	Mat lane ;
	lane = preProcess(src);
	Mat ROI;
	View(lane).copyTo(ROI);
	vector<Vec4i> lines;
	lines = fillLane(ROI); //ROI changed
	left = Mat(ROI.rows, ROI.cols, CV_8UC1, Scalar(0,0,0));
	right = Mat(ROI.rows, ROI.cols, CV_8UC1, Scalar(0,0,0));
	bool isL, isR;
	Mat onlyLane;
	left_right_lanes(ROI, lines,right,left,isL,isR).copyTo(onlyLane);
	imshow("onlyLane", onlyLane);
	centerP = calCenterPoint2(onlyLane,left,right,isL,isR);
}
Point DetectLane::getPoint()
{
	return centerP;
}
