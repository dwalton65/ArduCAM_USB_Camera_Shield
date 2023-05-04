// ArduCam_test.cpp : Defines the entry point for the console application.
//
#ifdef linux
#include "Arducam_SDK/ArduCamLib.h"
#include <unistd.h>
#include <termios.h>
#endif

#ifdef _WIN32
#include "stdafx.h"
#include <Windows.h>
#include "ArduCamlib.h"
#include <io.h>
#include <direct.h> 
#endif
#include <opencv2/opencv.hpp>
#include <thread>
#include <time.h>
#include <iostream>
#include <istream>
#include <string>

#include <sys/types.h> 
#include <sys/stat.h> 

#include <signal.h>

#include "Utils.h"

#include "arducam_config_parser.h"
#define USE_SOFT_TRIGGER

using namespace std;
using namespace cv;

ArduCamCfg cameraCfg;
volatile bool _running = true;
bool save_raw = false;
bool save_flag = true;

int color_mode = 0;

void showHelp(){
	printf(" usage: sudo ./ArduCam_Ext_Trigger_Demo <path/config-file-name>	\
			\n\n example: sudo ./ArduCam_Ext_Trigger_Demo ./../../cpp_config/AR0134_960p_Color.yml	\
			\n\n");
}


long total_frames[16];
void getAndDisplaySingleFrame(ArduCamHandle handle,int index){
	
    printf("Take picture.\n");
	char name[50];
	sprintf(name,"ArduCam%d",index);
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	
	ArduCamOutData* frameData;
	
	cv::Mat rawImage ;

	Uint32 rtn_val = ArduCam_getSingleFrame(handle, frameData);

	if ( rtn_val == USB_CAMERA_NO_ERROR) {
		rawImage = ConvertImage(frameData, color_mode);
		if (!rawImage.data)
		{
			std::cout << "Convert image fail,No image data \n";
			return;
		}

		total_frames[index]++;
		if(save_flag){
			char save_path[50];

			sprintf(save_path,"images%d",index);
#ifdef linux
			if(access(save_path, F_OK) != 0){  
				if(mkdir(save_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)  
					printf("mkdir error!\n");    
			}
#endif
#ifdef _WIN32
			if (_access(save_path, 0) != 0)
			{
				if (_mkdir(save_path))
					printf("mkdir error!\n");
			}
#endif
			printf("Camera%d,save image%ld.jpg.\n",index,total_frames[index]);
			char imageName[50];
			sprintf(imageName,"images%d/image%ld.jpg",index,total_frames[index]);

			if(save_raw){
				char rawName[50];
				sprintf(rawName,"images%d/image%ld.raw",index,total_frames[index]);
				FILE *file = fopen(rawName,"w");
				fwrite(frameData->pu8ImageData,1,cameraCfg.u32Width * cameraCfg.u32Height,file);
				fclose(file);
			}

			cv::imwrite(imageName,rawImage);
		}
		
		cv::resize(rawImage,rawImage,cv::Size(640, 480), (0, 0), (0, 0), cv::INTER_LINEAR);
	    cv::imshow(name, rawImage);
		//cvWaitKey(50);
	    cv::waitKey(50);
		printf("End display.\n");
	}else{
		printf("Take picture fail,ret_val = %d\n",rtn_val);
	}

}

void signal_handle(int signal){
	if(SIGINT == signal){
		_running = false;
	}
#ifdef linux
    usleep(1000 * 500);
#elif _WIN32
	Sleep(500);
#endif
    exit(0);
}


int main(int argc,char **argv)
{
	//receive Ctrl + C signal
	signal(SIGINT,signal_handle);
#ifdef linux
	static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
#endif
	//get config file name
	const char * config_file_name;
	if(argc > 1){
		config_file_name = argv[1];
	}else{
		showHelp();
		return 0;
	}


	ArduCamIndexinfo pUsbIdxArray[16];
	int camera_num = ArduCam_scan(pUsbIdxArray);

    printf("device num:%d\n",camera_num);
	char serial[16];
	unsigned char *u8TmpData;
    for(int i = 0; i < camera_num ;i++){
		u8TmpData = pUsbIdxArray[i].u8SerialNum;
		sprintf(serial,"%c%c%c%c-%c%c%c%c-%c%c%c%c",
			u8TmpData[0], u8TmpData[1], u8TmpData[2], u8TmpData[3],
			u8TmpData[4], u8TmpData[5], u8TmpData[6], u8TmpData[7],
			u8TmpData[8], u8TmpData[9], u8TmpData[10], u8TmpData[11]);
        printf("index:%4d\tSerial:%s\n",pUsbIdxArray[i].u8UsbIndex,serial);
    }
	
#ifdef linux
    sleep(2);
#endif
	
	printf("Found %d devices.\n",camera_num);
	ArduCamHandle cameraHandles[16];
	long sTriggerSendTime[16];

	for(int i = 0 ; i < camera_num ;i++){
		//read config file and open the camera.
		sTriggerSendTime[i] = 0;
		if (!camera_initFromFile(config_file_name, cameraHandles[i], cameraCfg, color_mode, i)) 
			 cameraHandles[i] = NULL;
		else{
			Uint32 ret_val =  ArduCam_setMode(cameraHandles[i],EXTERNAL_TRIGGER_MODE);
            if(ret_val == USB_BOARD_FW_VERSION_NOT_SUPPORT_ERROR){
                printf("Usb board firmware version not support single mode.\n");
                return 1;
            }
		}
	}

	while(_running && camera_num > 0){
			
		for(int i = 0 ; i < camera_num ;i++){
			ArduCamHandle tempHandle = cameraHandles[i];
			if(tempHandle == NULL){
				continue;
			}
			Uint32 rtn_val = ArduCam_isFrameReady(tempHandle);
			// printf("-----%d\n",rtn_val);
			if(rtn_val == 1){

#ifdef USE_SOFT_TRIGGER
				sTriggerSendTime[i] = 0;
#endif
				getAndDisplaySingleFrame(tempHandle,i);
			}
#ifdef USE_SOFT_TRIGGER
			else if(time(NULL) - sTriggerSendTime[i] > 3){
				ArduCam_softTrigger(tempHandle);
				sTriggerSendTime[i] = time(NULL);
			}
#endif
		}
		usleep( 1000 * 50);
		//cvWaitKey(10);
	}
	
	cv::destroyAllWindows();
	
	for(int i = 0 ;i < camera_num ;i++){
		if(cameraHandles[i] != NULL){
			ArduCam_close(cameraHandles[i]);
		}
	}
	std::cout << std::endl << "Press ENTER to exit..." << std::endl;
	std::string str_key;
	std::getline(std::cin,str_key);
#ifdef linux
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
#endif
	return 0;
}


