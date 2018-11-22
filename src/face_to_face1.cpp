#include "ros/ros.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
using namespace ros;

cv_bridge::CvImagePtr img_org = NULL;

void callback(const sensor_msgs::Image::ConstPtr& msg)
{
	try{
		img_org = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}catch (...){
		img_org = NULL;
	}
}

void detect_face(void)
{
	auto img = img_org;//この関数の途中でcallbackが別の画像にimg_orgを切り替える可能性があるので参照をコピー
	if(img == NULL)
		return;

	cv::Vec3b p = img->image.at<cv::Vec3b>(0,0);//左上の画素
	ROS_INFO("B: %d, G: %d, R: %d", p(0), p(1), p(2));//がその値を表示してみる
}

int main(int argc, char **argv)
{
	init(argc,argv,"face_to_face");
	NodeHandle n("~");

	Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, callback);

	int freq = 10;
	Rate loop_rate(freq);
	unsigned int c = 0;
	while(ok()){
		detect_face();
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}
