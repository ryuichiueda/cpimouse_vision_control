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

void detect_face(Publisher *pub_face)
{
	auto img = img_org; //この関数の途中でcallbackが別の画像にimg_orgを切り替える可能性があるので参照をコピー
	if(img == NULL)
		return;

	cv_bridge::CvImage gimg;
	cv::cvtColor(img->image, gimg.image, CV_BGR2GRAY); //顔検出用白黒画像を作る

	std::vector<cv::Rect> faces; //顔の座標を入れるベクトル
	cv::CascadeClassifier cascade("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"); //検出器
	cascade.detectMultiScale(gimg.image, faces, 1.1, 1, CV_HAAR_FIND_BIGGEST_OBJECT); //検出実行

	if(faces.size() == 0){ //顔が検出されないときは元の画像を/faceに飛ばす
		pub_face->publish(img->toImageMsg());
		return;
	}

	auto r = faces[0]; //第一候補の座標をrに入れる
	cv::rectangle(img->image, cv::Point(r.x, r.y),
			cv::Point(r.x+r.width, r.y+r.height), cv::Scalar(0,255,255),3); //元の画像に黄色い枠を描画
	pub_face->publish(img->toImageMsg()); // /faceに枠付きの画像を飛ばす
}

int main(int argc, char **argv)
{
	init(argc,argv,"face_to_face");
	NodeHandle n("~");

	Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, callback);
	Publisher pub_face = n.advertise<sensor_msgs::Image>("/face",1);

	int freq = 10;
	Rate loop_rate(freq);
	unsigned int c = 0;
	while(ok()){
		detect_face(&pub_face);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}
