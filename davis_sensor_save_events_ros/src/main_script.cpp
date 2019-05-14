#include "common.h"
#include "parser.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;

class Communicator{
	public:
		Communicator():sync_(mySyncPolicy(10)){
			if(Parser::hasOption("-help")){
				std::cout << "Optional -o: Output folder path (default: ros package location).\n"
					"Example: rosrun xtionpro_sensor_save_images xtionpro_sensor_save_images -o /path/to/output/" << std::endl;
			}
			
			// Make folders for saving current image
			std::string save_loc;
			if(Parser::hasOption("-o")){
				save_loc = Parser::getStringOption("-o");
			}else{
				std::string pwd = ros::package::getPath("xtionpro_sensor_save_images_ros");
				save_loc = pwd + "/dataset/";
			}

			folder_name_rgb = save_loc + "rgb/";
			folder_name_depth = save_loc + "depth/";

			std::string folder_remove_command;
			folder_remove_command = "rm -rf " + folder_name_depth;
			system(folder_remove_command.c_str());
			folder_remove_command = "rm -rf " + folder_name_rgb;
			system(folder_remove_command.c_str());

			std::string folder_create_command;
			folder_create_command = "mkdir -p " + folder_name_depth;
			system(folder_create_command.c_str());
			folder_create_command = "mkdir -p " + folder_name_rgb;
			system(folder_create_command.c_str());

			// Make rgb and depth filename log
			rgb_log.open((save_loc + "rgb.txt").c_str());
			depth_log.open((save_loc + "depth.txt").c_str());
			asc_log.open((save_loc + "associations.txt").c_str());
			vicon_log.open((save_loc + "groundtruth.txt").c_str());

			rgb_log << "# color images" << std::endl;
			rgb_log << "# file" << std::endl;
			rgb_log << "# timestamp filename" << std::endl;

			depth_log << "# depth images" << std::endl;
			depth_log << "# file" << std::endl;
			depth_log << "# timestamp filename" << std::endl;

			vicon_log << "# time x y z qx qy qz qw vx vy vz wx wy wz" << std::endl;

			ROS_INFO("file is successfully opened.");

			sub_depth.subscribe( nh_, "/camera/depth/image_rect", 1 );
			sub_rgb.subscribe( nh_, "/camera/rgb/image_rect_color", 1 );
//			sub_vicon = nh_.subscribe<nav_msgs::Odometry>("/vicon/lsi_asus/lsi_asus", 10, &Communicator::callback_vicon, this);
			sub_vicon = nh_.subscribe<geometry_msgs::TransformStamped>("/vicon/lsi_asus/lsi_asus", 10, &Communicator::callback_vicon, this);

			sync_.connectInput(sub_rgb, sub_depth);
			sync_.registerCallback(boost::bind(&Communicator::callback_asus, this, _1, _2));

			ROS_INFO("initialize ROS");
		}
		~Communicator(){
			if( rgb_log.is_open() ) rgb_log.close();
			if( depth_log.is_open() ) depth_log.close();
			if( asc_log.is_open() ) asc_log.close();
			if( vicon_log.is_open() ) vicon_log.close();

			ROS_INFO("file is successfully closed.");
		}
		void callback_asus(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);
		void callback_vicon(const nav_msgs::Odometry::ConstPtr& msg_vicon);
		void callback_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_vicon;
		message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
		message_filters::Subscriber<sensor_msgs::Image> sub_depth;
		message_filters::Synchronizer<mySyncPolicy> sync_;
		std::ofstream rgb_log, depth_log, asc_log, vicon_log;
		std::string folder_name_rgb, folder_name_depth;
};

void Communicator::callback_asus(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth){
	cv_bridge::CvImagePtr img_ptr_rgb, img_ptr_depth;

	try{
		img_ptr_rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
		img_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << std::setprecision(6) << std::fixed << ros::Time::now().toSec();

	cv::Mat color = img_ptr_rgb->image;
	cv::Mat depth = img_ptr_depth->image*1000;
	depth.convertTo(depth, CV_16UC1);

	std::string image_file_name;
	image_file_name = folder_name_depth + time.str() + ".png";
	cv::imwrite(image_file_name, depth);
	std::cout << image_file_name << std::endl;

	image_file_name = folder_name_rgb + time.str() + ".png";
	cv::imwrite(image_file_name, color);
	std::cout << image_file_name << std::endl;

	rgb_log << time.str() << " rgb/" << time.str() << ".png" << std::endl;
	depth_log << time.str() << " depth/" << time.str() << ".png" << std::endl;
	asc_log << time.str() << " rgb/" << time.str() << ".png " << time.str() << " depth/" << time.str() << ".png" << std::endl;
}

void Communicator::callback_vicon(const nav_msgs::Odometry::ConstPtr& msg_vicon){
	
	std::stringstream time;
	time << std::setprecision(6) << std::fixed << ros::Time::now().toSec();
	
	vicon_log << std::setprecision(10) << std::fixed
		<< time.str() << '\t'
		<< msg_vicon->pose.pose.position.x << '\t'
		<< msg_vicon->pose.pose.position.y << '\t'
		<< msg_vicon->pose.pose.position.z << '\t'
		<< msg_vicon->pose.pose.orientation.x << '\t'
		<< msg_vicon->pose.pose.orientation.y << '\t'
		<< msg_vicon->pose.pose.orientation.z << '\t'
		<< msg_vicon->pose.pose.orientation.w << '\t'
		<< msg_vicon->twist.twist.linear.x << '\t'
		<< msg_vicon->twist.twist.linear.y << '\t'
		<< msg_vicon->twist.twist.linear.z << '\t'
		<< msg_vicon->twist.twist.angular.x << '\t'
		<< msg_vicon->twist.twist.angular.y << '\t'
		<< msg_vicon->twist.twist.angular.z << std::endl;
}

void Communicator::callback_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon){
	
	std::stringstream time;
	time << std::setprecision(6) << std::fixed << ros::Time::now().toSec();
	
	vicon_log << std::setprecision(10) << std::fixed
		<< time.str() << '\t'
		<< msg_vicon->transform.translation.x << '\t'
		<< msg_vicon->transform.translation.y << '\t'
		<< msg_vicon->transform.translation.z << '\t'
		<< msg_vicon->transform.rotation.x << '\t'
		<< msg_vicon->transform.rotation.y << '\t'
		<< msg_vicon->transform.rotation.z << '\t'
		<< msg_vicon->transform.rotation.w << std::endl;
}

int main(int argc, char * argv[]){

	ros::init(argc, argv, "xtionpro_sensor_save_images");
	Parser::init(argc, argv);

	Communicator comm_;

	while( ros::ok() ) {
		ros::spinOnce();
		if( cv::waitKey(1) == 'q') break;
	}

	return 0;
}
