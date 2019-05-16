#include "common.h"
#include "parser.h"

class Communicator{
	public:
		Communicator(){
			if(Parser::hasOption("-help")){
				std::cout << "Optional -o: Output folder path (default: ros package location).\n"
					"Example: rosrun davis_sensor_save_events davis_sensor_save_events -o /path/to/output/" << std::endl;
			}
			
			// Make folders for saving current image
			std::string save_loc;
			if(Parser::hasOption("-o")){
				save_loc = Parser::getStringOption("-o");
			}else{
				std::string pwd = ros::package::getPath("davis_sensor_save_events_ros");
				save_loc = pwd + "/dataset/";
			}

			folder_name_image = save_loc + "image/";

			std::string folder_remove_command;
			folder_remove_command = "rm -rf " + folder_name_image;
			system(folder_remove_command.c_str());

			std::string folder_create_command;
			folder_create_command = "mkdir -p " + folder_name_image;
			system(folder_create_command.c_str());

			// Make image filename log
			image_log.open((save_loc + "images.txt").c_str());
			event_log.open((save_loc + "events.txt").c_str());
			imu_log.open((save_loc + "imu.txt").c_str());
			vicon_log.open((save_loc + "groundtruth.txt").c_str());

			image_log << "# gray images" << std::endl;
			image_log << "# timestamp filename" << std::endl;

			event_log << "# events" << std::endl;
			event_log << "# timestamp x y polarity" << std::endl;

			imu_log << "# imu" << std::endl;
			imu_log << "# acceleration gyroscope" << std::endl;
			imu_log << "# timestamp ax ay az gx gy gz" << std::endl;

			vicon_log << "# time x y z qx qy qz qw vx vy vz wx wy wz" << std::endl;
			ROS_INFO("file is successfully opened.");

			sub_image = nh_.subscribe<sensor_msgs::Image>("/image", 1, &Communicator::callback_aps, this );
			sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu", 10, &Communicator::callback_imu, this );
			sub_event = nh_.subscribe<dvs_msgs::EventArray>("/event", 10, &Communicator::callback_dvs, this );
			sub_vicon = nh_.subscribe<geometry_msgs::TransformStamped>("/vicon", 10, &Communicator::callback_vicon, this );

			ROS_INFO("initialize ROS");
		}
		~Communicator(){
			if( image_log.is_open() ) image_log.close();
			if( event_log.is_open() ) event_log.close();
			if( imu_log.is_open() ) imu_log.close();
			if( vicon_log.is_open() ) vicon_log.close();

			ROS_INFO("file is successfully closed.");
		}
		void callback_aps(const sensor_msgs::Image::ConstPtr& msg_image);
		void callback_imu(const sensor_msgs::Imu::ConstPtr& msg_imu);
		void callback_dvs(const dvs_msgs::EventArray::ConstPtr& msg_event);
		void callback_vicon(const nav_msgs::Odometry::ConstPtr& msg_vicon);
		void callback_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_vicon, sub_image, sub_imu, sub_event;
		std::ofstream image_log, event_log, imu_log, vicon_log;
		std::string folder_name_image;
};

void Communicator::callback_aps(const sensor_msgs::ImageConstPtr& msg_image){
	cv_bridge::CvImagePtr img_ptr_image;

	try{
		img_ptr_image = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << std::setprecision(6) << std::fixed << ros::Time::now().toSec();

	cv::Mat image = img_ptr_image->image;

	std::string image_file_name;
	image_file_name = folder_name_image + time.str() + ".png";
	cv::imwrite(image_file_name, image);

	image_log << time.str() << " image/" << time.str() << ".png" << std::endl;
}

void Communicator::callback_dvs(const dvs_msgs::EventArray::ConstPtr& msg_event){

	if( msg_event->header.stamp.sec != 0 ){
		for( unsigned int e = 0; e < msg_event->events.size(); e++ ){
			event_log << std::setprecision(10) << std::fixed
				<< msg_event->events[e].ts << '\t'
				<< msg_event->events[e].x << '\t'
				<< msg_event->events[e].y << '\t'
				<< msg_event->events[e].polarity+0 << std::endl;
		}
	}
}

void Communicator::callback_imu(const sensor_msgs::Imu::ConstPtr& msg_imu){
	
	std::stringstream time;
	time << std::setprecision(6) << std::fixed << ros::Time::now().toSec();
	
	imu_log << std::setprecision(10) << std::fixed
		<< time.str() << '\t'
		<< msg_imu->linear_acceleration.x << '\t'
		<< msg_imu->linear_acceleration.y << '\t'
		<< msg_imu->linear_acceleration.z << '\t'
		<< msg_imu->angular_velocity.x << '\t'
		<< msg_imu->angular_velocity.y << '\t'
		<< msg_imu->angular_velocity.z << std::endl;
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

	ros::init(argc, argv, "davis_sensor_save_images");
	Parser::init(argc, argv);

	Communicator comm_;

	while( ros::ok() ) {
		ros::spinOnce();
		if( cv::waitKey(1) == 'q') break;
	}

	return 0;
}
