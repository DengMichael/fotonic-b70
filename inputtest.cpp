#include <libusb-1.0/libusb.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <fstream>
#include <bitset>
#include <thread>
#include <chrono>
using namespace std;

#define EP_OUT 0x02
#define EP_IN 0x81
#define EP_OTHER 0x83

#define VID 0x0525
#define PID 0xA9A0

#define COMMAND_BUFFER_SIZE 436
#define COMMAND_RESPONSE_BUFFER_SIZE 436

#define FRAME_BUFFER_SIZE 77924
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

void user_loop(int &get_frames)
{
	do
	{
		cout << "Press x to stop image stream" << endl;
	} while (cin.get() != 'x');
	get_frames = 0;
	return;
}

class frames
{
public:
	cv::Mat *b_display_frame, *z_display_frame, *x_display_frame, *y_display_frame;
	frames(cv::Mat *input_b_frame, cv::Mat *input_z_frame, cv::Mat *input_x_frame, cv::Mat *input_y_frame);
};

frames::frames(cv::Mat *input_b_frame, cv::Mat *input_z_frame, cv::Mat *input_x_frame, cv::Mat *input_y_frame)
{
	b_display_frame = input_b_frame;
	z_display_frame = input_z_frame;
	x_display_frame = input_x_frame;
	y_display_frame = input_y_frame;
}

static void LIBUSB_CALL
xfr_cb_out(struct libusb_transfer *transfer)
{
	int *completed = (int *)transfer->user_data;
	short *frame = (short *)transfer->buffer;
	short b_frame[19200];
	short z_frame[19200];
	short x_frame[19200];
	short y_frame[19200];

	//skip first row (664) - then 4s afterwards.
	for (int y = 0; y < INUMY; y++)
	{
		int iRowPosB = ((y + 1) * INUMX) * 4 + (INUMX * 0); //640
		int iRowPosZ = ((y + 1) * INUMX) * 4 + (INUMX * 1);
		int iRowPosX = ((y + 1) * INUMX) * 4 + (INUMX * 2);
		int iRowPosY = ((y + 1) * INUMX) * 4 + (INUMX * 3); //77280
		//		cout << "posy:" << iRowPosY << endl;
		for (int x = 0; x < INUMX; x++)
		{
			// row format: Bx160, Zx160, Xx160, Yx160 (16bit per component)
			int frameIndex = y * INUMX + x;
			b_frame[frameIndex] = frame[4 * (y + 2) + iRowPosB + x]; // 8
			//code says
			// scale by 10, will give 1.0 = 1.0 centimeter
			z_frame[frameIndex] = frame[4 * (y + 2) + iRowPosZ + x];
			x_frame[frameIndex] = frame[4 * (y + 2) + iRowPosX + x];
			y_frame[frameIndex] = frame[4 * (y + 2) + iRowPosY + x]; //77923
			if (frameIndex == 19199)
			{
				cout << "end" << endl;
				cout << frameIndex << endl;
				cout << 4 * (y + 2) + iRowPosY + x << endl;
				cout << y_frame[frameIndex] << endl;
				cout << y_frame[19199] << endl;
				cout << frame[4 * (y + 2) + iRowPosY + x] << endl;
				cout << frame[77923] << endl;
			}
		}
	}
	//apply a mask to remove the 0 values - then alter

	pair<short *, short *> minmaxB = minmax_element(begin(b_frame), end(b_frame));
	cout << "The min element in B is " << *(minmaxB.first) << '\n';
	cout << "The max element in B is " << *(minmaxB.second) << '\n';
	pair<short *, short *> minmaxZ = minmax_element(begin(z_frame), end(z_frame));
	cout << "The min element in Z is " << *(minmaxZ.first) << '\n';
	cout << "The max element in Z is " << *(minmaxZ.second) << '\n';
	pair<short *, short *> minmaxX = minmax_element(begin(x_frame), end(x_frame));
	cout << "The min element in X is " << *(minmaxX.first) << '\n';
	cout << "The max element in X is " << *(minmaxX.second) << '\n';
	pair<short *, short *> minmaxY = minmax_element(begin(y_frame), end(y_frame));
	cout << "The min element in Y is " << *(minmaxY.first) << '\n';
	cout << "The max element in Y is " << *(minmaxY.second) << '\n';

	cv::Mat imageframe = cv::Mat(INUMY_WITH_ADDITIONAL_ROW, FOUR_INUMX_WITH_BUFFER, CV_16UC1, frame); //hmmm
	// get min/max values using OpenCV min_max function
	double minVal, maxVal;
	cv::minMaxLoc(imageframe, &minVal, &maxVal);
	//	cout << "pre-norm min/max = " << minVal << " " << maxVal << " " << endl;
	cv::minMaxLoc(imageframe, &minVal, &maxVal);
	//	cout << "post-norm min/max = " << minVal << " " << maxVal << " " << endl;
	cv::Mat for_display_only;
	imageframe.convertTo(for_display_only, CV_8UC1);
	cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

	cv::Mat b_display_frame = cv::Mat(INUMY, INUMX, CV_16UC1, b_frame); //hmmm
	// get min/max values using OpenCV min_max function
	minVal, maxVal;
	cv::minMaxLoc(b_display_frame, &minVal, &maxVal);
	cout << "pre-norm min/maxb = " << minVal << " " << maxVal << " " << endl;
	cv::normalize(b_display_frame, b_display_frame, 0, 65280, cv::NORM_MINMAX);
	cv::minMaxLoc(b_display_frame, &minVal, &maxVal);
	cout << "post-norm min/maxb = " << minVal << " " << maxVal << " " << endl;
	cv::namedWindow("B", cv::WINDOW_NORMAL);

	cv::Mat z_display_frame = cv::Mat(INUMY, INUMX, CV_16UC1, z_frame); //hmmm
	// get min/max values using OpenCV min_max function
	minVal, maxVal;
	cv::minMaxLoc(z_display_frame, &minVal, &maxVal);
	cout << "pre-norm min/maxz = " << minVal << " " << maxVal << " " << endl;
	//	cv::normalize(z_display_frame, z_display_frame, 0, 1, cv::NORM_MINMAX);
	cv::minMaxLoc(z_display_frame, &minVal, &maxVal);
	cout << "post-norm min/maxz = " << minVal << " " << maxVal << " " << endl;
	cv::Mat z_for_display_only;
	z_display_frame.convertTo(z_for_display_only, CV_8UC1);
	cv::namedWindow("Z", cv::WINDOW_NORMAL);

	//8 bit frame
	//logarithmic
	//exponential
	//histogram
	//contrast stretch

	//write as pngs

	cv::Mat x_display_frame = cv::Mat(INUMY, INUMX, CV_16UC1, x_frame); //hmmm
	// get min/max values using OpenCV min_max function
	minVal, maxVal;
	cv::minMaxLoc(x_display_frame, &minVal, &maxVal);
	cout << "pre-norm min/maxx = " << minVal << " " << maxVal << " " << endl;
	cv::normalize(x_display_frame, x_display_frame, 0, 1, cv::NORM_MINMAX);
	cv::minMaxLoc(x_display_frame, &minVal, &maxVal);
	cout << "post-norm min/maxx = " << minVal << " " << maxVal << " " << endl;
	cv::namedWindow("X", cv::WINDOW_NORMAL);

	cv::Mat y_display_frame = cv::Mat(INUMY, INUMX, CV_16UC1, y_frame); //hmmm
	// get min/may values using OpenCV min_may function
	minVal, maxVal;
	cv::minMaxLoc(y_display_frame, &minVal, &maxVal);
	cout << "pre-norm min/maxy = " << minVal << " " << maxVal << " " << endl;
	cv::normalize(y_display_frame, y_display_frame, 0, 1, cv::NORM_MINMAX);
	cv::minMaxLoc(y_display_frame, &minVal, &maxVal);
	cout << "post-norm min/maxy = " << minVal << " " << maxVal << " " << endl;
	cv::namedWindow("Y", cv::WINDOW_NORMAL);

	cv::imshow("test", for_display_only);
	cv::imshow("B", b_display_frame);
	cv::imshow("Z", z_for_display_only);
	cv::imshow("X", x_display_frame);
	cv::imshow("Y", y_display_frame);
	//	cv::imshow("calcHist Demo", histImage);

	//the fps issue - how to resolve it
	//i need to run a thread that synchronizes everything properly
	//time from the
	cv::waitKey(1);

	*completed = 1;
}

void printarray(unsigned short *input, int n)
{
	// loop through the elements of the array
	for (int i = 0; i < n; i++)
	{
		std::cout << input[i] << " " << i << std::endl;
	}
	std::cout << input[0] << " first" << std::endl;
	std::cout << input[1] << " second" << std::endl;

	return;
}

unsigned short stringtounsignedshort(string input)
{
	unsigned short output = (unsigned short)strtoul(input.c_str(), NULL, 16);
	return output;
}

unsigned short swapendianunsignedshort(unsigned short input)
{
	unsigned short endian = (input << 8) | (input >> 8);
	return endian;
}

unsigned short *hexstringtoarray(std::string input)
{
	int n = input.size();
	printf("\n%i index\n", n);
	unsigned short *output = new unsigned short[n];
	for (int index = 0; index < n; index += 4)
	{
		string bytestrings = input.substr(index, 4);
		output[index / 4] = swapendianunsignedshort(stringtounsignedshort(bytestrings));
	}
	return output;
}

void fill_command(unsigned short *command_buffer, unsigned short *command_counter, unsigned short command, unsigned short parameter)
{
	const static int command_counter_location = 0;
	const static int command_location = 2;
	const static int parameter_location = 8; //15
	cout << *command_counter << endl;
	cout << command << endl;
	cout << parameter << endl;
	//fill by entering code
	command_buffer[command_location] = command;
	command_buffer[parameter_location] = parameter;
	command_buffer[command_counter_location] = *command_counter;
	return;
}

//takes required libusb, buffer, and command, send to device
void send_command(unsigned short *command_buffer, unsigned short *command_counter, unsigned short command, unsigned short parameter, libusb_device_handle *dev_handle)
{
	//convert
	memset(command_buffer, 0, COMMAND_BUFFER_SIZE);
	int command_transferred = 0;
	fill_command(command_buffer, command_counter, command, parameter);
	printarray(command_buffer, 436);
	int status = libusb_bulk_transfer(
		dev_handle,
		EP_OUT,
		(unsigned char *)command_buffer, //unsigned char*
		COMMAND_BUFFER_SIZE,			 //unsigned unsigned short
		&command_transferred,			 //int*
		0);
	printf("Transferred: %i\n", command_transferred);
	*command_counter += 1;
	cout << "sent command" << endl;
}

void recieve_command_response(unsigned short *command_response_buffer, int command_response_transferred, libusb_device_handle *dev_handle)
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
	printf("Transferred: %i\n", command_response_transferred);
	cout << "recieved command response" << endl;
}

//parameter is probably the wrong type
//assume command is in the right format

int main(void)
{
	unsigned short command_buffer[436];
	int command_transferred = 0;
	unsigned short command_counter = 9;
	unsigned short parameter = 3;
	unsigned short command_response_buffer[436];
	int command_response_transferred;

	static struct libusb_device_handle *dev_handle = NULL;
	static struct libusb_device *dev;
	static struct libusb_device **devs;
	static struct libusb_context *mContext;
	unsigned int i;
	int status = 0;
	//

	libusb_init(&mContext);
	//	libusb_set_debug(mContext, 4);
	int rc;
	libusb_get_device_list(NULL, &devs);
	int completed = 0;
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

		cout << "hiya" << endl;
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
			//
			libusb_control_transfer(dev_handle,
									0,
									11,
									1,
									0,
									0, //data
									0,
									0);
			printf("Reset: %i\n", rc);
			rc = libusb_clear_halt(dev_handle,
								   EP_IN);
			printf("Clear: %i\n", rc);
			rc = libusb_clear_halt(dev_handle,
								   EP_OUT);
			printf("Clear: %i\n", rc);
			rc = libusb_clear_halt(dev_handle,
								   EP_OTHER);
			printf("Clear: %i\n", rc);

			int transferred;
			std::cout << "libusb_get_device_descriptor(" << std::endl;

			//set up a buffer to feed inputs
			//another buffer to get outputs

			//function to fill buffer with command and value

			//			unsigned short *data = hexstringtoarray("01000000024000000000840000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			unsigned short *data = hexstringtoarray("01000000024000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			unsigned short data1[218];
			int transferred1;
			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);
			//			data = hexstringtoarray("02000000084002000000000000000000050000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			//			data = hexstringtoarray("02000000084000000000000000000000050000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			//setmode
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("03000000034002000000000000000000901012000000000015000000084002000000000000000000050000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);
			//it's just a trace of the old commands for tracking????????
			//but sometimes they just disappear
			data = hexstringtoarray("04000000054002000000000000000000280000000000000000fe1200000000001600000003400200000000000000000064001200000000001500000008400200000000000000000005000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("050000002a4002000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("060000000340040000000000000000006400320000000000180000002a4002000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("0700000030400200000000000000000020030000000000000000000000000000190000000340040000000000000000006400320000000000180000002a400200000000000000000001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("08000000324002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			printarray(data, 436);

			//so i guess i can remove everything past that point
			//data = hexstringtoarray("0900000006400200000000000000000001000000000000001b000000324002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

			send_command(command_buffer, &command_counter, CMD_DE_SET_LSSWITCH, 1, dev_handle);
			recieve_command_response(command_response_buffer, command_response_transferred, dev_handle);

			//data = hexstringtoarray("09000000064000000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			/*
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			printarray(data, 436);
			//

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);
			*/

			data = hexstringtoarray("0a00000003400200000000000000000020030000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500000015000100000000000000000000000000400100f03f010000300100000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("0b0000000540020000000000000000002800000000000000b8fe1200000000001d00000003400200000000000000000020030000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("0c0000002a40020000000000000000000100000000000000b8fe1200000000001d00000003400200000000000000000020030000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("0d0000000540020000001200000000003c00000003400200000000000000000020030000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e806150000001500010000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);
			////////////////////////////////////////

			data = hexstringtoarray("0e00000008400200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500000015000100000000000000000000000000400100f03f010000300100000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("0f0000000540020000000000000000003c00000000000000b8fe1200000000002100000008400200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("100000002a40020000000000000000000100000000000000b8fe1200000000002100000008400200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("1100000037400a000000000000000000010002000000b10001600000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500000015000100000000000000000000000000400100f03f01");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("12000000304002000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500000015000100000000000000000000000000400100f03f0100003001000000000000000020040000480e0000ffffffff");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			data = hexstringtoarray("13000000324002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bcf91200b7fc907c080000000100000000000000080000002cfa1200c2fd907c00000000000000000000140000000000c4fa120008000000ddfd907c01000000ffffffff000000000000000000d0fd7fccfa120000100000000000000000000000001400000000000000000034fa1200e0f91200eada907ce82d917c18000000a0fd1200a0fd120001000000ffffffff000000007cfe12004406817c4adb907c8706817cf0000000a306817cd84d3d00e7c2437e212c557800000000000000009f0100014b00000000000000000015000301000000b0fd7f20040000480e0000010000000800000000000000e8061500000015000100000000000000000000000000400100f03f0100003001000000000000000020040000480e0000ffffffff");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			//

			data = hexstringtoarray("14000000014000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);

			short frame[FRAME_BUFFER_SIZE];
			short frame2[FRAME_BUFFER_SIZE];

			struct libusb_transfer *transfer;
			//			struct libusb_transfer *transfer2;
			transfer = libusb_alloc_transfer(0);
			//			transfer2 = libusb_alloc_transfer(0);
			//

			//data structure with 4 buffers

			//
			libusb_fill_bulk_transfer(transfer, dev_handle, EP_OTHER, (unsigned char *)&frame, sizeof(frame), xfr_cb_out, &completed, 0); //&completed
			//			libusb_fill_bulk_transfer(transfer2, dev_handle, EP_OTHER, (unsigned char *)&frame2, sizeof(frame2), xfr_cb_out, &completed, 0);

			int get_frames = 1;

			thread user_input(user_loop, ref(get_frames));
			//this is where i start threading i suppose
			//i guess i make an image display thread instead
			//i pass the matrices to my bulk transfer - letting the callback function edit them
			//my image display thread updates, every x values
			/* Handle Events */

			/*

			//pass to the bulk transfers
			//create 4 matrices, etc, i keep filling those buffers in,
			//meanwhile my imshow thread recieves the buffer and reads it

			*/

			while (get_frames)
			{
				//submit a transfer,
				//convert it to rain
				//i think the best way to is to read multiple streams.
				//add a stream, they'll all use the same callback function - so it should be fine?
				//just test it
				libusb_submit_transfer(transfer);
				cout << "hello" << endl;
				//imshow here

				cout << "hello2" << endl;
				while (!completed)
				{
					rc = libusb_handle_events_completed(NULL, &completed);
					if (rc != LIBUSB_SUCCESS)
					{
						fprintf(stderr, "Transfer Error: %s\n", libusb_error_name(rc));
						break;
					}
				}
				completed = 0;
			}

			//////////////////// THEORETICAL END
			//join here
			user_input.join();
			libusb_free_transfer(transfer);
			//			libusb_free_transfer(transfer2);

			//stop camera
			data = hexstringtoarray("14000000024000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
			status = libusb_bulk_transfer(
				dev_handle,
				EP_OUT,
				(unsigned char *)data, //unsigned char*
				436,				   //unsigned unsigned short
				&transferred,		   //int*
				0);
			printf("Transferred: %i\n", transferred);

			status = libusb_bulk_transfer(
				dev_handle,
				EP_IN,
				(unsigned char *)&data1, //unsigned char*
				sizeof(data1),			 //unsigned unsigned short
				&transferred1,			 //int*
				0);
			printf("Transferred: %i\n", transferred1);
		};
		libusb_release_interface(dev_handle, 0);
		libusb_close(dev_handle);
		libusb_exit(NULL);
		return 0;
	};
};
