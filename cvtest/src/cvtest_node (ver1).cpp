#include <ros/ros.h>
#include <cvtest/cv_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>

class cvcalc{
 public:
  cvcalc();
  ~cvcalc();
  void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);  //とりあえずmatに画像を入れるcallback
  void calcmoment();      //画像処理をして緑部分の重心を検出
  void judge_enemy();     //緑が見えているか判定、位置をはく。
  cv::Point2f cog;        //重心の画像内の座標

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat mat;
  int width_of_image;
  bool status;            //緑が見えていたらtrueになったらいいな
  bool isdoubleget;       //２つ見えているかどうか。widthのほうが大きいとtrue
  double place;           //カメラのセンターが0、左端が-1、右端が1になったらいいな
  double width_of_rectangle;
  double height_of_rectangle;
};

cvcalc::cvcalc(){
 sub = nh.subscribe<sensor_msgs::Image>("/red_bot/camera/image_raw", 1, &cvcalc::rgbImageCallback, this);
 pub = nh.advertise<cvtest::cv_msg>("cvtest", 100);
 status = false;
 isdoubleget = false;
 place = 0;
 width_of_rectangle = 0;
 height_of_rectangle = 0;
}

cvcalc::~cvcalc(){
}

void cvcalc::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
 cv_bridge::CvImagePtr cv_ptr;
 try{
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
 }catch(cv_bridge::Exception& ex){
  ROS_ERROR("error!");
  exit(-1);
 }
 mat = cv_ptr->image;
 width_of_image = mat.cols;
}

void cvcalc::calcmoment(){
 cv::Mat filtered_image;
 std::vector<cv::Mat> planes;
 std::vector<cv::Vec3f> circles;
 cv::split(mat, planes);
 cv::threshold(planes[1], planes[1], 130, 255, CV_THRESH_BINARY);     //１：緑が130以上のところを抽出
 cv::threshold(planes[0], planes[0], 10, 255, CV_THRESH_BINARY_INV);  //２：赤が10以下のところを抽出
 cv::bitwise_and(planes[0],planes[1],filtered_image);                 //３：上の2つの論理積を取る。ほぼ緑単色が抽出できればいいな
// cv::Moments mu = cv::moments(filtered_image, false);                 //モーメントの計算、よくわからない。
// cog = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);                     //とりあえずこうすれば重心座標が取れるらしい
// circle(mat, cog, 4, cv::Scalar(100), 2, 4);                          //matへの表示用。重心に小さい丸を打つ。
// std::cout<<cog.x<<","<<cog.y<<std::endl;
 cv::Rect rc = cv::boundingRect(filtered_image);
 std::cout << rc.x << ":" << rc.width << ":" << rc.y << ":" << rc.height << std::endl;
 rectangle(mat, cv::Point2f(rc.x, rc.y), cv::Point2f(rc.x + rc.width, rc.y + rc.height),cv::Scalar(100), 2, 4);
//いろいろ代入
 cog = cv::Point2f(rc.x + rc.width / 2, rc.y + rc.height / 2);
 width_of_rectangle = rc.width;
 height_of_rectangle = rc.height;
//いろいろ代入終わり
circle(mat, cog, 4, cv::Scalar(100), 2, 4);

// imshow("planes1", planes[1]);      //１：の部分
// imshow("planes0", planes[0]);      //２：の部分
 imshow("filtered", filtered_image);  //３：の部分
 imshow("cvtest2", mat);
 cv::waitKey(10);
}

void cvcalc::judge_enemy(){
 if (std::isnan(cog.x)){
  status = false;
  place = 0;
 }else{
  status = true;
  place = cog.x / (width_of_image / 2) - 1;
 }
 if (width_of_rectangle > height_of_rectangle + 10){
   isdoubleget = true;
 }else{
   isdoubleget = false;
 }
 std::cout<<status<<","<<place<<std::endl;
 std::cout<<width_of_rectangle<<","<<height_of_rectangle<<","<<isdoubleget<<std::endl;
}

int main(int argc, char** argv){
 ros::init(argc, argv ,"cvtest_node");
 cvcalc cvcalc;
 ros::Rate rate(5);
 sleep(3);                //ないとやばそうなエラーが出る。多分画像が出力されていない状態でmatを読みに行ってしまう系
 while (ros::ok()){
  ros::spinOnce();
  cvcalc.calcmoment();
  cvcalc.judge_enemy();
  rate.sleep();
 }
 return 0;
}
