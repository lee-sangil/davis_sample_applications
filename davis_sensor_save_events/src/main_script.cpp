#include <libcaercpp/devices/davis.hpp>

#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using namespace std;

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if (signal == SIGTERM || signal == SIGINT) {
		globalShutdown.store(true);
	}
}

static void usbShutdownHandler(void *ptr) {
	(void) (ptr); // UNUSED.

	globalShutdown.store(true);
}

int main(int argc, char * argv[]) {
	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce a help screen")
		("event,e", "display events")
		("frame,f", "display APS frames")
		("quiet,q", "run the exe in quiet mode")
		;

	boost::program_options::variables_map vm;
	boost::program_options::store(parse_command_line(argc, argv, desc), vm);

	if(vm.count("help"))
	{
		std::cout << "Usage: regex [options]\n";
		std::cout << desc;
		return (EXIT_SUCCESS);
	}

// Install signal handler for global shutdown.
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#else
	struct sigaction shutdownAction;

	shutdownAction.sa_handler = &globalShutdownSignalHandler;
	shutdownAction.sa_flags   = 0;
	sigemptyset(&shutdownAction.sa_mask);
	sigaddset(&shutdownAction.sa_mask, SIGTERM);
	sigaddset(&shutdownAction.sa_mask, SIGINT);

	if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// Make folders for saving current image
	std::string folder_name_img = "log/images/";

	std::string folder_remove_command;
	folder_remove_command = "rm -rf " + folder_name_img;
	if( system(folder_remove_command.c_str()) ){
		return (EXIT_FAILURE);
	}

	std::string folder_create_command;
	folder_create_command = "mkdir -p " + folder_name_img;
	if( system(folder_create_command.c_str()) ){
		return (EXIT_FAILURE);
	}

	// Open file for writing.
	std::fstream fEventOutput, fIMUOutput, fImageOutput;;
	fEventOutput.open("log/events.txt", std::fstream::out);
	fImageOutput.open("log/images.txt", std::fstream::out);
	fIMUOutput.open("log/imu.txt", std::fstream::out);

	// Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	libcaer::devices::davis davisHandle = libcaer::devices::davis(1);

	// Let's take a look at the information we have on the device.
	struct caer_davis_info davis_info = davisHandle.infoGet();
	std::cout << davis_info.deviceString
		<< " --- ID: " << davis_info.deviceID 
		<< ", Master: " << davis_info.deviceIsMaster 
		<< ", DVS X:" << davis_info.dvsSizeX 
		<< ", DVS Y: " << davis_info.dvsSizeY 
		<< ", Logic: " << davis_info.logicVersion << std::endl;

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	davisHandle.sendDefaultConfig();

	// Now let's get start getting some data from the device. We just loop in blocking mode,
	// no notification needed regarding new events. The shutdown notification, for example if
	// the device is disconnected, should be listened to.
	davisHandle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	std::cout << "Started logging to file ..." << std::endl;

	char image_index_char[255];
	std::string image_file_name;
	unsigned int image_index = 0;

	while (!globalShutdown.load(memory_order_relaxed)) {
		std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davisHandle.dataGet();
		if (packetContainer == nullptr) {
			continue; // Skip if nothing there.
		}

		for (auto &packet : *packetContainer) {
			if (packet == nullptr) {
				continue; // Skip if nothing there.
			}

			if (packet->getEventType() == POLARITY_EVENT) {
				std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
					= std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

				cv::Mat cvEvents(davis_info.dvsSizeY, davis_info.dvsSizeX, CV_8UC3, cv::Vec3b{127, 127, 127});

				// Print out timestamps and addresses.
				for (const auto &evt : *polarity) {
					int64_t ts = evt.getTimestamp64(*polarity);
					uint16_t x = evt.getX();
					uint16_t y = evt.getY();
					bool pol   = evt.getPolarity();

					if( vm.count("event") ){
						cvEvents.at<cv::Vec3b>(y, x) = pol? cv::Vec3b{0, 0, 255} : cv::Vec3b{0, 255, 0};
					}

					fEventOutput << std::fixed << std::setprecision(6) << (double)ts*1e-6 << " " << x << " " << y << " " << pol << std::endl;
				}

				if( vm.count("event") ){
					cv::imshow("DVS-EVS", cvEvents);
					cv::waitKey(1);
				}
			}
			else if (packet->getEventType() == IMU6_EVENT) {
				std::shared_ptr<const libcaer::events::IMU6EventPacket> imu
					= std::static_pointer_cast<libcaer::events::IMU6EventPacket>(packet);

				// Print out timestamps and data.
				for (const auto &evt : *imu) {
					int64_t ts   = evt.getTimestamp64(*imu);
					float accelX = evt.getAccelX();
					float accelY = evt.getAccelY();
					float accelZ = evt.getAccelZ();
					float gyroX  = evt.getGyroX();
					float gyroY  = evt.getGyroY();
					float gyroZ  = evt.getGyroZ();

					fIMUOutput << std::fixed << std::setprecision(6) << (double)ts*1e-6 << " " << accelX << " " << accelY << " " << accelZ << " " << gyroX
							   << " " << gyroY << " " << gyroZ << std::endl;
				}
			}
			else if (packet->getEventType() == FRAME_EVENT) {
				std::shared_ptr<const libcaer::events::FrameEventPacket> frame
					= std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

				for (const auto &f : *frame) {
					if (f.getROIIdentifier() != 0) {
						continue;
					}

					cv::Mat cvFrame = f.getOpenCVMat(false);
					int32_t ts = f.getTimestamp();

					if( vm.count("frame") ){
							// Simple display, just use OpenCV GUI.
							cv::imshow("DVS-APS", cvFrame);
							cv::waitKey(1);
					}

					sprintf(image_index_char, "frame_%08d", image_index++);
					image_file_name = folder_name_img + image_index_char + ".png";
					cv::imwrite(image_file_name, cvFrame);

					if( !vm.count("quiet") ){
						std::cout << image_file_name << std::endl;
					}

					fImageOutput << std::fixed << std::setprecision(6) << (double)ts*1e-6 << " " << "images/" << image_index_char << ".png" << std::endl;
				}

			}
		}
	}

	davisHandle.dataStop();

	// Close automatically done by destructor.
	fEventOutput.close();
	fIMUOutput.close();
	fImageOutput.close();

	if( vm.count("frame") ) cv::destroyWindow("DVS-APS");
	if( vm.count("event") )	cv::destroyWindow("DVS-EVS");

	std::cout << "Stopped logging to file." << std::endl;

	return (EXIT_SUCCESS);
}
