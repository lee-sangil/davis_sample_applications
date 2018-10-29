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
					uint16_t x = evt.getX();
					uint16_t y = evt.getY();
					bool pol   = evt.getPolarity();

					if( vm.count("event") ){
						cvEvents.at<cv::Vec3b>(y, x) = pol? cv::Vec3b{0, 0, 255} : cv::Vec3b{0, 255, 0};
					}
				}

				if( vm.count("event") ){
					cv::imshow("DVS-EVS", cvEvents);
					cv::waitKey(1);
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

					if( vm.count("frame") ){
							// Simple display, just use OpenCV GUI.
							cv::imshow("DVS-APS", cvFrame);
							cv::waitKey(1);
					}
				}
			}
		}
	}

	davisHandle.dataStop();

	// Close automatically done by destructor.
	if( vm.count("frame") ) cv::destroyWindow("DVS-APS");
	if( vm.count("event") )	cv::destroyWindow("DVS-EVS");

	return (EXIT_SUCCESS);
}
