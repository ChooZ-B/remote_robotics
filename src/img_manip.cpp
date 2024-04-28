#include"img_manip.hpp"
IManip::~IManip(){};

/*
 *
 * ManipPrototype member function definitions
 *
 */

ManipPrototype::ManipPrototype(const char* s_topic, int sq_size, const char* p_topic, int pq_size)
    : it_(nh_)
{
    sub_ = it_.subscribe(s_topic,sq_size,&ManipPrototype::callback,this);
    pub_ = it_.advertise(p_topic,pq_size);
}

void ManipPrototype::callback(const sensor_msgs::ImageConstPtr& msg)
{
    pub_.publish(msg);
    return;
}

void ManipPrototype::manipulate(cv::Mat& image)
{
    return;
}

/*
 *
 * Manip member function definitions
 *
 */

Manip::Manip(const char* s_topic, int sq_size, const char* p_topic, int pq_size)
  : ManipPrototype(s_topic,sq_size,p_topic,pq_size){};

void Manip::callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n", e.what());
        return;
    }

    manipulate(cv_ptr->image);

    //check if manipulate changed image encoding
    if(cv_ptr->image.channels() == 1)
    {
        cv_ptr->encoding = "mono8";
    }
    else if(cv_ptr->image.channels() == 3)
    {
        cv_ptr->encoding = "bgr8";
    }

    pub_.publish(cv_ptr->toImageMsg());

    return;
}

void Manip::manipulate(cv::Mat& image)
{
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY); // simple grayscale
    return;
}

/*
 *
 *  ImgSegmentSimple member function definitions
 *
 */

ImgSegmentSimple::ImgSegmentSimple(const char* s_topic, int sq_size, const char* p_topic, int pq_size)
    : Manip(s_topic, sq_size, p_topic, pq_size)
{
    roi_.x = 0;
    roi_.y = 0;
    roi_.width = 0;
    roi_.height = 0;

    srv_ = nh_.advertiseService("rectangle/dimensions",&ImgSegmentSimple::set_dim,this);
}

void ImgSegmentSimple::manipulate(cv::Mat& image)
{
    if(((roi_.x + roi_.width)>image.cols) || ((roi_.y + roi_.height)>image.rows))
        ROS_INFO("Rectangle dimensions too large for image container.\n");
    else if(roi_.area() == 0)
        ; //do nothing to the image
    else
        image = image(roi_); // crop image according to roi dimensions
    return;
}

bool ImgSegmentSimple::set_dim(remote_robotics::rect_dims::Request& req, remote_robotics::rect_dims::Response& res)
{
    ROS_INFO("request x=%d, y=%d, width=%d, height=%d\n",req.X,req.Y,req.WIDTH,req.HEIGHT);
    if((req.X<0) || (req.Y<0) || (req.WIDTH<0) || (req.HEIGHT<0))
    {
        res.RESPONSE = std::string("ERROR: No negative coordinates allowed.");
        return true;
    }
    roi_.x = req.X;
    roi_.y = req.Y;
    roi_.width = req.WIDTH;
    roi_.height = req.HEIGHT;

    res.RESPONSE = std::string("OK");

    return true;
}

/*
 *
 * ImgSegmentSpecial member function definitions
 *
 */
ImgSegmentSpecial::ImgSegmentSpecial(const char* s_topic, int sq_size, const char* p_topic, int pq_size)
    : it_(nh_)//: Manip(s_topic,sq_size,p_topic,pq_size) 
{
    motor_rack_[0] = cv::Rect(840, 125, 120, 120);   //Servo 0
    motor_rack_[1] = cv::Rect(850, 240, 120, 120);   //Servo 1
    motor_rack_[2] = cv::Rect(850, 365, 120, 120);   //Servo 2
    motor_rack_[3] = cv::Rect(840, 480, 120, 120);   //Servo 3
    motor_rack_[4] = cv::Rect(735, 120, 120, 120);   //Servo 4
    motor_rack_[5] = cv::Rect(735, 245, 120, 120);   //Servo 5
    motor_rack_[6] = cv::Rect(735, 375, 120, 120);   //Servo 6
    motor_rack_[7] = cv::Rect(735, 500, 120, 120);   //Servo 7
    motor_rack_[8] = cv::Rect(615, 115, 120, 120);   //Servo 8
    motor_rack_[9] = cv::Rect(615, 245, 120, 120);   //Servo 9
    motor_rack_[10] = cv::Rect(615, 375, 120, 120);   //Servo 10
    motor_rack_[11] = cv::Rect(615, 510, 120, 120);   //Servo 11
    motor_rack_[12] = cv::Rect(480, 115, 120, 120);   //Servo 12
    motor_rack_[13] = cv::Rect(480, 245, 120, 120);   //Servo 13
    motor_rack_[14] = cv::Rect(480, 375, 120, 120);   //Servo 14
    motor_rack_[15] = cv::Rect(480, 510, 120, 120);   //Servo 15
    motor_rack_[16] = cv::Rect(345, 120, 120, 120);   //Servo 16
    motor_rack_[17] = cv::Rect(345, 245, 120, 120);   //Servo 17
    motor_rack_[18] = cv::Rect(345, 375, 120, 120);   //Servo 18
    motor_rack_[19] = cv::Rect(350, 500, 120, 120);   //Servo 19
    motor_rack_[20] = cv::Rect(235, 130, 120, 120);   //Servo 20
    motor_rack_[21] = cv::Rect(235, 250, 120, 120);   //Servo 21
    motor_rack_[22] = cv::Rect(235, 375, 120, 120);   //Servo 22
    motor_rack_[23] = cv::Rect(240, 490, 120, 120);   //Servo 23

    //roi_.x      = 0;
    //roi_.y      = 0;
    //roi_.width  = 0;
    //roi_.height = 0;

    motor_number = 24;

    sub_ = it_.subscribe(s_topic,sq_size,&ImgSegmentSpecial::callback,this);
    pub_ = nh_.advertise<remote_robotics::MotorImg>(p_topic,pq_size); //
    server_ = nh_.advertiseService("rectangle/index",&ImgSegmentSpecial::set_dim_by_index,this);
    ROS_INFO("image crop service is ready for processing requests\n");
    
}

void ImgSegmentSpecial::manipulate(cv::Mat& image)
{
    if((motor_number >= 0) && (motor_number <= 23))
        image = image(motor_rack_[motor_number]);
    return;
}

void ImgSegmentSpecial::callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    remote_robotics::MotorImg mi_msg;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n", e.what());
        return;
    }

    manipulate(cv_ptr->image);

    //check if manipulate changed image encoding
    if(cv_ptr->image.channels() == 1)
    {
        cv_ptr->encoding = "mono8";
    }
    else if(cv_ptr->image.channels() == 3)
    {
        cv_ptr->encoding = "bgr8";
    }
    mi_msg.IMG = *cv_ptr->toImageMsg(); ///! IMG of data type ImagePtr, does not compile
    mi_msg.INDEX = motor_number;

    pub_.publish(mi_msg);

    return;
}   
bool ImgSegmentSpecial::set_dim_by_index(remote_robotics::motor_idx::Request& req, remote_robotics::motor_idx::Response& res)
{
    ROS_INFO("request: motor index=%d\n",req.INDEX);
    if((req.INDEX<0)||(req.INDEX>23))
    {
        res.RESPONSE = std::string("ERROR: WRONG MOTOR INDEX");
    }
    else
    {
        motor_number = req.INDEX;
        //roi_ = motor_rack_[motor_number];
        res.RESPONSE = std::string("OK");
    }

    return true;
}

/*
 *
 * BinaryImg member function definitions
 *
 */
BinaryImg::BinaryImg(const char* s_topic, int sq_size, int pq_size)
    : Manip(s_topic,sq_size,"image/binary",pq_size){};

void BinaryImg::manipulate(cv::Mat& image)
{
    
    cv::Mat channels[3];

    cv::split(image,channels);

    cv::subtract(channels[2],channels[1],image);

    cv::threshold(image,image,35,255,cv::THRESH_BINARY);

    return;
}
