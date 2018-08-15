#include "ros/ros.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "signal.h"
#include <string>
#include <vector>
#include <opencv2/objdetect/objdetect.hpp>
/*
#include <opencv2/imgproc/imgproc.hpp>
   #include <opencv2/highgui/highgui.hpp>
   */
using namespace ros;

cv_bridge::CvImagePtr img_org = NULL;

void onSigint(int sig)
{
	std_srvs::Trigger trigger;
	service::call("/motor_off", trigger);
	shutdown();
}

void callback(const sensor_msgs::Image::ConstPtr& msg)
{
	try{
		img_org = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}catch (...){
		img_org = NULL;
	}
}

void one_step(Subscriber *sub, Publisher *pub)
{
	if(img_org == NULL)
		return;

	auto org = img_org;
	cv_bridge::CvImagePtr gimg;
	cv::cvtColor(org->image,gimg->image,CV_BGR2GRAY);
	cv::CascadeClassifier cascade("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");

	std::vector<cv::Rect> faces;
	cascade.detectMultiScale(gimg->image,faces,1.1,1,CV_HAAR_FIND_BIGGEST_OBJECT);

	if(faces.size() == 0){
		pub->publish(org->toImageMsg());
		return;
	}

	auto r = faces[0];
	cv::rectangle(org->image,cv::Point(r.x, r.y),
			cv::Point(r.x+r.width, r.y+r.height), cv::Scalar(0,255,255),3);
	pub->publish(org->toImageMsg());

	double wid = (double)org->image.cols/2;
	double pos_x_rate = (r.x + r.width/2 - wid)/wid;
	double rot = -pos_x_rate*3.141592/4;
	ROS_INFO("detected %f",rot);

	geometry_msgs::Twist tw;
	tw.linear.x = 0.0;
	tw.angular.z = rot;

	pub->publish(tw);

	return;
}

int main(int argc, char **argv)
{
	init(argc,argv,"face_to_face");
	NodeHandle n("~");

	Publisher pub = n.advertise<sensor_msgs::Image>("/face",1);
	Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, callback);

	service::waitForService("/motor_on");
	service::waitForService("/motor_off");

	signal(SIGINT, onSigint);

	int freq = 10;
	Rate loop_rate(freq);
	unsigned int c = 0;
	while(ok()){
		one_step(&sub, &pub);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

/*
#!/usr/bin/env python
#encoding: utf8
import rospy, cv2, math                         #mathを追加
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist             #追加
from std_srvs.srv import Trigger                #追加

class FaceToFace():
    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.pub = rospy.Publisher("face", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_org = None
        ###以下のモータの制御に関する処理を追加###
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
        rospy.ServiceProxy('/motor_on', Trigger).call()

    def monitor(self,rect,org):
        if rect is not None:
            cv2.rectangle(org,tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]),(0,255,255),4)
       
        self.pub.publish(self.bridge.cv2_to_imgmsg(org, "bgr8"))
   
    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_face(self):
        if self.image_org is None:
            return None
    
        org = self.image_org
    
        gimg = cv2.cvtColor(org,cv2.COLOR_BGR2GRAY)
        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        cascade = cv2.CascadeClassifier(classifier)
        face = cascade.detectMultiScale(gimg,1.1,1,cv2.CASCADE_FIND_BIGGEST_OBJECT)
    
        if len(face) == 0:    #len(face)...以下を次のように書き換え
            self.monitor(None,org)
            return None       
                              
        r = face[0]           
        self.monitor(r,org)   
        return r  

    def rot_vel(self):        #このメソッドを追加
        r = self.detect_face()
        if r is None:
            return 0.0
           
        wid = self.image_org.shape[1]/2   #画像の幅の半分の値
        pos_x_rate = (r[0] + r[2]/2 - wid)*1.0/wid
        rot = -0.25*pos_x_rate*math.pi    #画面のキワに顔がある場合にpi/4[rad/s]に
        rospy.loginfo("detected %f",rot)
        return rot

    def control(self):         #新たにcontrolメソッドを作る
        m = Twist()
        m.linear.x = 0.0
        m.angular.z = self.rot_vel()
        self.cmd_vel.publish(m)

if __name__ == '__main__':
    rospy.init_node('face_to_face')
    fd = FaceToFace()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fd.control()
        rate.sleep()

# Copyright 2016 Ryuichi Ueda
# Released under the MIT License.
# To make line numbers be identical with the book, this statement is written here. Don't move it to the header.
	*/
