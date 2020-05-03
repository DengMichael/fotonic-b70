#include <libusb-1.0/libusb.h>
#include <opencv2/opencv.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <cstring>
#include <fstream>
#include <bitset>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#define EP_OUT 0x02
#define EP_IN 0x81
#define EP_OTHER 0x83

#define NUMBER_OF_FRAME_TRANSFERS 10

#define VID 0x0525
#define PID 0xA9A0

#define COMMAND_BUFFER_SIZE 436
#define COMMAND_RESPONSE_BUFFER_SIZE 436

#define FRAME_BUFFER_SIZE 77924 * 2
#define FRAME_SIZE 19200

#define INUMX 160
#define INUMY 120
#define FOUR_INUMX_WITH_BUFFER 644 //160*4
#define INUMY_WITH_ADDITIONAL_ROW 121

//commands
//control operations
#define CMD_DE_SENSOR_START 0x4001
#define CMD_DE_SENSOR_STOP 0x4002

//set operation parameters
#define CMD_DE_SET_SHUTTER 0x4003
#define CMD_DE_SET_FPS 0x4005
#define CMD_DE_SET_LSSWITCH 0x4006
#define CMD_DE_SET_MODE 0x4008
#define CMD_DE_SET_EXPOSURE 0x400A //SET_SHUTTER, SET_CMR, SET_FPS combined
#define CMD_DE_SET_FPS_DIVISOR 0x402A

//set filter control and parameters
#define CMD_DE_SET_FILTERCONTROL 0x4101
#define CMD_DE_SET_LOWLIGHT_GT_PASSIVE 0x4102
#define CMD_DE_SET_LOWLIGHT_GT_ACTIVE 0x4103
#define CMD_DE_SET_SATURATION_LT_B 0x4104
#define CMD_DE_SET_SATURATION_LT_A_MINUS_B 0x4105
#define CMD_DE_SET_SATURATION_GT_A_MINUS_B 0x4106

//multishutter operation parameters
#define CMD_DE_SET_SHUTTER_EXT 0x4037
#define CMD_DE_SET_MS_SATURATION 0x4030
#define CMD_DE_SET_MS_TYPE 0x4032
#define CMD_DE_GET_MS_TYPE 0x4033

//--CMD PARAMETERS--

//CMD_DE_SET_MODE parameter
#define DE_MODE_TEMPORAL 0x0
#define DE_MODE_SPATIAL 0x1 //not currently working
#define DE_MODE_SPATIO_TEMPORAL 0x3
#define DE_MODE_MULTI_ST 0x5
#define DE_MODE_ZFAST 0x6
#define DE_MODE_ZFINE 0x7
#define DE_MODE_MS_ST 0xA
#define DE_MODE_BM_ZFINE 0xB	//experimental
#define DE_MODE_BM_TEMPORAL 0xC //experimental

//CMD_DE_SET_MS_TYPE parameter
#define DE_MS_TYPE_SATURATION 0x0
#define DE_MS_TYPE_WEIGHTED_AVG 0x1
#define DE_MS_TYPE_RATIO 0x2

std::mutex m;
std::condition_variable condv;
bool display_greyscale = true;
bool get_frames = true;
bool update_viewer = false;

//
int frames_obtained = 0;

auto t1 = std::chrono::high_resolution_clock::now();

float theta = M_PI; // The angle of rotation in radians
Eigen::Affine3f transform = Eigen::Affine3f::Identity();
pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialize_pointcloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_ptr->width = INUMX;
	cloud_ptr->height = INUMY;
	transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	return cloud_ptr;
}

pcl::visualization::PCLVisualizer::Ptr initialize_visualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
pcl::visualization::PCLVisualizer::Ptr viewer;

//4 matrices for every frame
cv::Mat frames[NUMBER_OF_FRAME_TRANSFERS][4];

void user_loop()
{
	std::string str;
	do
	{
		do
		{
			cout << "Enter x to stop image stream " << endl;
			cout << "Enter g to display greyscale" << endl;
			cout << "Enter c to display color" << endl;
			cout << "Enter b to change background to black" << endl;
			cout << "Enter w to change background to white" << endl;
			getline(cin, str);
		} while (str != "x" && str != "g" && str != "c" && str != "b" && str != "w");
		if (str == "g")
		{
			display_greyscale = true;
		}
		else if (str == "c")
		{
			display_greyscale = false;
		}
		else if (str == "b")
		{
			viewer->setBackgroundColor(0, 0, 0);
		}
		else if (str == "w")
		{
			viewer->setBackgroundColor(255, 255, 255);
		}
	} while (str != "x");
	update_viewer = false;
	get_frames = false;
	//cancel all transfers
	return;
}

int current_callback_packet = 0;
int frame_to_process = 0;
void spin_loop()
{
	unsigned iActiveBrightnessVMin = 20;
	unsigned iActiveBrightnessVMax = 340;
	while (get_frames)
	{
		std::unique_lock<std::mutex> lk(m);
		condv.wait(lk, [] { return update_viewer; }); //wait for callback
		if (current_callback_packet == 0)
		{
			frame_to_process = NUMBER_OF_FRAME_TRANSFERS - 1;
		}
		else
		{
			frame_to_process = current_callback_packet - 1;
		}
		point_cloud_ptr->clear();

		if (display_greyscale)
		{
			for (int y = 0; y < INUMY; y++)
			{
				for (int x = 0; x < INUMX; x++)
				{
					pcl::PointXYZRGB point;
					point.z = (float)frames[frame_to_process][1].at<short>(y, x) * 0.1f;
					point.x = (float)frames[frame_to_process][2].at<short>(y, x) * 0.1f;
					point.y = (float)frames[frame_to_process][3].at<short>(y, x) * 0.1f;

					int iTmpB = (frames[frame_to_process][0].at<short>(y, x) * 0.1f) - iActiveBrightnessVMin;
					iTmpB = (int)(iTmpB / ((iActiveBrightnessVMax - iActiveBrightnessVMin) / 256.0));
					if (iTmpB < 0)
						iTmpB = 0;
					if (iTmpB > 255)
						iTmpB = 255;
					//r = 0-127
					//g = 0-127 then 128 to 255
					//b = 128 to 255
					point.r = iTmpB;
					point.g = iTmpB;
					point.b = iTmpB;

					point_cloud_ptr->points.push_back(point);
				}
			}
		}
		else
		{
			for (int y = 0; y < INUMY; y++)
			{
				for (int x = 0; x < INUMX; x++)
				{
					pcl::PointXYZRGB point;
					point.z = (float)frames[frame_to_process][1].at<short>(y, x) * 0.1f;
					point.x = (float)frames[frame_to_process][2].at<short>(y, x) * 0.1f;
					point.y = (float)frames[frame_to_process][3].at<short>(y, x) * 0.1f;
					int iTmpB = (frames[frame_to_process][0].at<short>(y, x) * 0.1f) - iActiveBrightnessVMin;
					iTmpB = (int)(iTmpB / ((iActiveBrightnessVMax - iActiveBrightnessVMin) / 256.0));
					if (iTmpB < 0)
						iTmpB = 0;
					if (iTmpB > 255)
						iTmpB = 255;
					//r = 0-127
					//g = 0-127 then 128 to 255
					//b = 128 to 255
					if (iTmpB < 127)
					{
						point.r = 255 - (iTmpB * 2);
						point.g = iTmpB * 2;
						point.b = 0;
					}
					else
					{
						point.r = 0;
						point.g = (255 - (iTmpB - 128) * 2); //as b increases, g decreases at twice that rate.
						point.b = (iTmpB - 128) * 2;
					}
					point_cloud_ptr->points.push_back(point);
				}
			}
		}

		pcl::transformPointCloud(*point_cloud_ptr, *point_cloud_ptr, transform);
		viewer->updatePointCloud(point_cloud_ptr, "cloud");

		cv::normalize(frames[frame_to_process][0], frames[frame_to_process][0], -32768, 32768, cv::NORM_MINMAX);
		cv::normalize(frames[frame_to_process][1], frames[frame_to_process][1], -32768, 32768, cv::NORM_MINMAX);
		cv::normalize(frames[frame_to_process][2], frames[frame_to_process][2], -32768, 32768, cv::NORM_MINMAX);
		cv::normalize(frames[frame_to_process][3], frames[frame_to_process][3], -32768, 32768, cv::NORM_MINMAX);

		cv::imshow("B", frames[frame_to_process][0]);
		cv::imshow("Z", frames[frame_to_process][1]);
		cv::imshow("X", frames[frame_to_process][2]);
		cv::imshow("Y", frames[frame_to_process][3]);
		viewer->spinOnce();
		update_viewer = false;

		cv::waitKey(1);
	}
	return;
}

static void LIBUSB_CALL xfr_cb_out(struct libusb_transfer *transfer)
{
	frames_obtained += 1;
	unsigned short *frame_buffer = (unsigned short *)transfer->buffer;
	//skip first row (664) - then 4s afterwards.

	for (int y = 0; y < INUMY; y++)
	{
		int iRowPosB = ((y + 1) * INUMX) * 4 + (INUMX * 0);
		int iRowPosZ = ((y + 1) * INUMX) * 4 + (INUMX * 1);
		int iRowPosX = ((y + 1) * INUMX) * 4 + (INUMX * 2);
		int iRowPosY = ((y + 1) * INUMX) * 4 + (INUMX * 3);
		for (int x = 0; x < INUMX; x++)
		{
			int iColumnPos = 4 * (y + 2) + x;
			// row format: Bx160, Zx160, Xx160, Yx160 (16bit per component)
			frames[current_callback_packet][0].at<short>(y, x) = frame_buffer[iRowPosB + iColumnPos];
			frames[current_callback_packet][1].at<short>(y, x) = frame_buffer[iRowPosZ + iColumnPos];
			frames[current_callback_packet][1].at<short>(y, x) = (frames[current_callback_packet][1].at<short>(y, x) == 65535) ? 0 : frames[current_callback_packet][1].at<short>(y, x);
			//code says
			// scale by 10, will give 1.0 = 1.0 centimeter
			frames[current_callback_packet][2].at<short>(y, x) = frame_buffer[iRowPosX + iColumnPos];
			frames[current_callback_packet][3].at<short>(y, x) = frame_buffer[iRowPosY + iColumnPos];
		}
	}
	if (current_callback_packet < NUMBER_OF_FRAME_TRANSFERS - 1)
	{
		current_callback_packet += 1;
	}
	else
	{
		current_callback_packet = 0;
	}
	update_viewer = true;
	condv.notify_one(); //tell spin_loop to render

	/*

	cv::minMaxLoc(frames[1], &minVal, &maxVal);
	cout << "pre-norm min/maxz = " << minVal << " " << maxVal << " " << endl;
	cv::minMaxLoc(frames[1], &minVal, &maxVal);
	cout << "post-norm min/maxz = " << minVal << " " << maxVal << " " << endl;
	cv::minMaxLoc(frames[2], &minVal, &maxVal);
	//	cout << "pre-norm min/maxx = " << minVal << " " << maxVal << " " << endl;
	cv::minMaxLoc(frames[2], &minVal, &maxVal);
	//	cout << "post-norm min/maxx = " << minVal << " " << maxVal << " " << endl;

	cv::minMaxLoc(frames[3], &minVal, &maxVal);
	//	cout << "pre-norm min/maxy = " << minVal << " " << maxVal << " " << endl;
	cv::minMaxLoc(frames[3], &minVal, &maxVal);
	//	cout << "post-norm min/maxy = " << minVal << " " << maxVal << " " << endl;
	*/

	if (get_frames)
	{
		libusb_submit_transfer(transfer);
		if (frames_obtained == 10000)
		{
			auto t2 = std::chrono::high_resolution_clock::now();
			std::cout << "f() took "
					  << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
					  << " milliseconds\n";
			frames_obtained = 0;

			t1 = std::chrono::high_resolution_clock::now();
		}
	}

	//write as pngs

	return;
}

void fill_command(unsigned short (&command_buffer)[COMMAND_BUFFER_SIZE], unsigned short &command_counter, unsigned short command, unsigned short parameter)
{
	const static int command_counter_location = 0;
	const static int command_location = 2;
	const static int parameter_location = 8;
	command_buffer[command_location] = command;
	command_buffer[parameter_location] = parameter;
	command_buffer[command_counter_location] = command_counter;
	return;
}

//takes required libusb, buffer, and command, send to device
void send_command(unsigned short (&command_buffer)[COMMAND_BUFFER_SIZE], unsigned short &command_counter, unsigned short command, unsigned short parameter, libusb_device_handle *dev_handle)
{
	//convert
	memset(command_buffer, 0, COMMAND_BUFFER_SIZE);
	int command_transferred = 0;
	fill_command(command_buffer, command_counter, command, parameter);
	int status = libusb_bulk_transfer(
		dev_handle,
		EP_OUT,
		(unsigned char *)command_buffer, //unsigned char*
		COMMAND_BUFFER_SIZE,			 //unsigned unsigned short
		&command_transferred,			 //int*
		0);
	printf("Sent %i bytes\n", command_transferred);

	command_counter += 1;
}

void recieve_command_response(unsigned short (&command_response_buffer)[COMMAND_RESPONSE_BUFFER_SIZE], int command_response_transferred, libusb_device_handle *dev_handle)
{
	memset(command_response_buffer, 0, COMMAND_RESPONSE_BUFFER_SIZE);
	command_response_transferred = 0;
	int status = libusb_bulk_transfer(
		dev_handle,
		EP_IN,
		(unsigned char *)command_response_buffer, //unsigned char*
		COMMAND_RESPONSE_BUFFER_SIZE,			  //unsigned unsigned short
		&command_response_transferred,			  //int*
		0);
	printf("Recieved %i\n", command_response_transferred);
}

int main()
{

	static struct libusb_device_handle *dev_handle = NULL;
	static struct libusb_device *dev;
	static struct libusb_device **devs;
	static struct libusb_context *mContext;
	unsigned int i;
	int status = 0;

	libusb_init(&mContext);
	//	libusb_set_debug(mContext, 4);
	int rc;
	libusb_get_device_list(NULL, &devs);
	for (i = 0; (dev = devs[i]) != NULL; i++)
	{
		struct libusb_device_descriptor desc;
		status = libusb_get_device_descriptor(dev, &desc);
		printf("Descriptor: %i\n", status);

		if (status != 0)
		{
			std::cout << "libusb_get_device_descriptor(" << std::endl;
			return 1;
		}
		cout << desc.idVendor << endl;

		if ((VID == desc.idVendor) && (PID == desc.idProduct))
		{
			printf("Product: %x\n", desc.idVendor);
			status = libusb_open(dev, &dev_handle);
			printf("Open: %i\n", status);
			libusb_free_device_list(devs, 1);
			rc = libusb_set_auto_detach_kernel_driver(dev_handle, 1);
			printf("Detach: %i\n", rc);
			rc = libusb_claim_interface(dev_handle, 0);
			printf("Interface: %i\n", rc);
			libusb_set_interface_alt_setting(dev_handle, 0, 1);

			int transferred;
			std::cout << "libusb_get_device_descriptor(" << std::endl;

			//set up a buffer to feed inputs
			//another buffer to get outputs

			//function to fill buffer with command and value
			unsigned short command_buffer[436];
			int command_transferred = 0;
			unsigned short command_counter = 9;
			unsigned short parameter = 3;
			unsigned short command_response_buffer[436];
			int command_response_transferred;

			send_command(command_buffer, command_counter, CMD_DE_SENSOR_STOP, 0, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_MODE, DE_MODE_MULTI_ST, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_SHUTTER, 100, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_FPS, 40, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_FPS_DIVISOR, 1, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			//Set shutter again for multi-shutter apparently?
			send_command(command_buffer, command_counter, CMD_DE_SET_SHUTTER, 100, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_MS_SATURATION, 800, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_MS_TYPE, DE_MS_TYPE_SATURATION, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			send_command(command_buffer, command_counter, CMD_DE_SET_LSSWITCH, 1, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			// This is where custom settings are set in the original.

			libusb_transfer *transfers[NUMBER_OF_FRAME_TRANSFERS];
			short frame_buffers[NUMBER_OF_FRAME_TRANSFERS][FRAME_BUFFER_SIZE];

			cv::namedWindow("B", cv::WINDOW_NORMAL);
			cv::namedWindow("Z", cv::WINDOW_NORMAL);
			cv::namedWindow("X", cv::WINDOW_NORMAL);
			cv::namedWindow("Y", cv::WINDOW_NORMAL);

			//callback is sequential
			point_cloud_ptr = initialize_pointcloud();
			viewer = initialize_visualizer(point_cloud_ptr);

			for (int i = 0; i < NUMBER_OF_FRAME_TRANSFERS; i++)
			{
				frames[i][0] = cv::Mat(INUMY, INUMX, CV_16SC1);
				frames[i][1] = cv::Mat(INUMY, INUMX, CV_16SC1);
				frames[i][2] = cv::Mat(INUMY, INUMX, CV_16SC1);
				frames[i][3] = cv::Mat(INUMY, INUMX, CV_16SC1);

				transfers[i] = libusb_alloc_transfer(0);
				libusb_fill_bulk_transfer(transfers[i], dev_handle, EP_OTHER, (unsigned char *)&(frame_buffers[i]), sizeof(frame_buffers[i]), xfr_cb_out, NULL, 0);
			}
			cout << "Hi3" << endl;

			std::thread input_thread(user_loop);
			std::thread spin_thread(spin_loop);

			t1 = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < NUMBER_OF_FRAME_TRANSFERS; i++)
			{
				libusb_submit_transfer(transfers[i]);
			}

			send_command(command_buffer, command_counter, CMD_DE_SENSOR_START, 0, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			while (get_frames)
			{
				rc = libusb_handle_events(NULL);
				if (rc != LIBUSB_SUCCESS)
				{
					fprintf(stderr, "Transfer Error: %s\n", libusb_error_name(rc));
					break;
				}
			}

			input_thread.join();
			spin_thread.join();
			//free all transfers
			for (int i = 0; i < 10; i++)
			{
				libusb_free_transfer(transfers[i]);
			}

			send_command(command_buffer, command_counter, CMD_DE_SENSOR_STOP, 0, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);
		};
		libusb_release_interface(dev_handle, 0);
		libusb_close(dev_handle);
		libusb_exit(NULL);
		return 0;
	};
};
