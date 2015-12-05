#include "optdrone.h"




using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="color Image";
static const char WINDOW3[]="opt Image";
static const std_msgs::Empty e;
static int tag_timer = 0;
static int sim_timer = 0;
static int tag = 1;
static const int T = 27;
FILE * out;
FILE * xyz;


class my_ardrone_node{

    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    image_transport::Subscriber image_sub;
    ros::Subscriber nav_sub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist fresh_vel;
    ros::Publisher vel_pub;
    Mat last_image;//上一张图片
    Mat current_image;//下一张图片
    int x;//ardrone x
    int y;//ardrone y
    int z;//ardrone z
    int theta;//ardrone theta
    bool flag;
    void takeoff(){
        takeoff_pub.publish(e);

    }
    void land(){
        land_pub.publish(e);
    }
    void forward(float i){
        cmd_vel = fresh_vel;
        cmd_vel.linear.x = i;
        vel_pub.publish(cmd_vel);
    }
    void back(float i){
        cmd_vel = fresh_vel;

        cmd_vel.linear.x = -i;
        vel_pub.publish(cmd_vel);

    }
    void rotation_clock(float i){
        cmd_vel = fresh_vel;

        cmd_vel.angular.z = i;
        vel_pub.publish(cmd_vel);

    }
    void rotation_inclock(float i){
        cmd_vel = fresh_vel;

        cmd_vel.angular.z = -i;
        vel_pub.publish(cmd_vel);

    }
	void fly(float x, float y, float z, float rx, float ry, float rz){
		cmd_vel = fresh_vel;
		cmd_vel.linear.x = x;
		cmd_vel.linear.y = y;
		cmd_vel.linear.z = z;
		cmd_vel.angular.x = rx;
		cmd_vel.angular.y = ry;
		cmd_vel.angular.z = rz;
		vel_pub.publish(cmd_vel);
	}
    void process(const sensor_msgs::ImageConstPtr& cam_image){
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
          cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception:%s",e.what());
		  
          return;
        }
        /*Mat img_rgb = cv_ptr->image;
        Mat img_gray;
        resize(img_rgb,img_rgb,Size(0,0),0.5,0.5,CV_INTER_LINEAR);
        cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
        //cout << "nihao" << endl;
		
        imshow(WINDOW,img_rgb);
        imshow(WINDOW2,img_gray);
        last_image = img_rgb;*/
		//strategic,issf参数传递
		current_image = cv_ptr->image;
		if(!last_image.empty()){
			IplImage temp_curr = IplImage(current_image);
			IplImage* curr_iplimg = &temp_curr;
      			IplImage temp_last = IplImage(last_image);
			IplImage* last_iplimg = &temp_last;
			Mat frame_dst, color;
		        resize(current_image,frame_dst,Size(WIDTH,HEIGHT),0,0,1);
			IplImage temp_dst = IplImage(frame_dst);
			IplImage* dst_iplimg = &temp_dst;
		    color.create(HEIGHT, WIDTH, CV_8UC3);
            //farneback
			//float result = matStrategic(FarneBack, last_image, current_image, frame_dst, color, 7, false) / 100;
			//LK
			float result = imgStrategic(Lucaskanade, last_iplimg, curr_iplimg, dst_iplimg, color, 7) / 100;
			//HS
			//float result = imgStrategic(HornSchunck, last_iplimg, curr_iplimg, dst_iplimg, color, 7) / 100;
			//BM
			//float result = imgStrategic(BlockMatch, last_iplimg, curr_iplimg, dst_iplimg, color, 7) / 100;
			//PyrLK 
			//float result = imgFeatureStrategic(PyrLK, last_iplimg, curr_iplimg, dst_iplimg, color, 7) / 100;
			
            imshow(WINDOW,current_image);
            imshow(WINDOW2,color);
            imshow(WINDOW3,frame_dst);
	    fprintf(xyz, "%d, %d, %d, %d, %lf\n", x, y, z, theta, result);
			/*result 转化为6个自由度*/
		
        if(result == -2){
			tag_timer++;
			fprintf(out, "%d=====>\n", tag_timer);
			if(tag_timer > T && tag == 1){
				//tag_timer = 0;
 				tag = -1;
			}
			fly(0, 0, 0, 0, 0, tag);
	    }else if(result >= -0.3 && result <= 0.3){
			sim_timer++;
			if(sim_timer > 7){
				tag = 1;
				sim_timer = 0;
				tag_timer = 0;
			}
            fly(0.5,0,0,0,0,-result*0.1);
	    }else if(result >= -0.6 && result <= 0.6){
			sim_timer++;
			if(sim_timer > 7){
				tag = 1;
				sim_timer = 0;
				tag_timer = 0;
			}
			fly(0.2,0,0,0,0,-result*0.1);	
		}else{
			sim_timer++;
			if(sim_timer > 7){
				tag = 1;
				sim_timer = 0;
				tag_timer = 0;
			}
			fly(0.1, 0, 0, 0, 0, -result*0.1);
        }

        }
        last_image = current_image;

        int k = waitKey(1);
        if(k != -1)
            cout << k << endl;

        if(k == SPACE)//space for takeoff
            takeoff();
        if(k == ENTER)//enter for land
            land();
        if(k == UP)//up for forward
            forward(1);
        if(k == DOWN)//down for back
            back(1);
        if(k == LEFT)//left for turn_left
            rotation_clock(1);
        if(k == RIGHT)//right for turn_right
            rotation_inclock(1);
        return;

        }
    void nav_callback(ardrone_autonomy::Navdata msg){
        theta = msg.rotZ;
    }
    void odom_callback(nav_msgs::Odometry msg){
        x = msg.pose.pose.position.x;
        y = msg.pose.pose.position.y;
        z = msg.pose.pose.position.z;


    }
public:
    /*
     *init the subscribers and publishers
     */
    my_ardrone_node(){

        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
        image_sub = it.subscribe("/ardrone/image_raw",1,&my_ardrone_node::process,this);
        takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
        land_pub = n.advertise<std_msgs::Empty>("/ardrone/land",1);
        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        nav_sub = n.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata",1,&my_ardrone_node::nav_callback,this);
        odom_sub = n.subscribe<nav_msgs::Odometry>("ardrone/odom",1,&my_ardrone_node::odom_callback,this);
    }



};



int main(int argc, char **argv){
	ros::init(argc,argv,"opt_drone");
    cv::namedWindow(WINDOW);
    cv::namedWindow(WINDOW2);
    cv::namedWindow(WINDOW3);
    my_ardrone_node man;

	out = fopen("out","w");
	xyz = fopen("xyz", "w");
	ros::spin();
	return 0;
	}
