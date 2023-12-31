/*
* Copyright (c) 2017-2018 CelePixel Technology Co. Ltd. All Rights Reserved
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "celex5_ros.h"

static const std::string OPENCV_WINDOW_0 = "Master";
static const std::string OPENCV_WINDOW_1 = "Slave";

#ifdef _WIN32
#include <windows.h>
#else
#include<unistd.h>
#include <signal.h>
#endif

#define FPN_PATH_M    "/home/guoyeye/celex_ws/src/celex5_stereo_multithread/celex5dependencies/stereo/config/FPN_M_2.txt"
#define FPN_PATH_S    "/home/guoyeye/celex_ws/src/celex5_stereo_multithread/celex5dependencies/stereo/config/FPN_S_2.txt"

CeleX5 *pCeleX5 = new CeleX5;

#ifdef _WIN32
bool exit_handler(DWORD fdwctrltype)
{
	switch (fdwctrltype)
	{
		//ctrl-close: confirm that the user wants to exit.
	case CTRL_CLOSE_EVENT:
	case CTRL_C_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		delete pCeleX5;
		pCeleX5 = NULL;
		return(true);
	default:
		return false;
	}
}
#else
void exit_handler(int sig_num)
{
	printf("SIGNAL received: num =%d\n", sig_num);
	if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 || sig_num == 15)
	{
		delete pCeleX5;
		pCeleX5 = NULL;
		exit(0);
	}
}
#endif

int main()
{
	if (pCeleX5 == NULL)
		return 0;

	pCeleX5->openSensor(CeleX5::CeleX5_MIPI);
	pCeleX5->setFpnFile(FPN_PATH_M, 0);
	pCeleX5->setFpnFile(FPN_PATH_S, 1);

	CeleX5::CeleX5Mode sensorMode = CeleX5::Full_Picture_Mode;
	pCeleX5->setSensorFixedMode(sensorMode, 0);
	pCeleX5->setSensorFixedMode(sensorMode, 1);

#ifdef _WIN32
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)exit_handler, true);
#else
	// install signal use sigaction
	struct sigaction sig_action;
	sigemptyset(&sig_action.sa_mask);
	sig_action.sa_flags = 0;
	sig_action.sa_handler = exit_handler;
	sigaction(SIGHUP, &sig_action, NULL);  // 1
	sigaction(SIGINT, &sig_action, NULL);  // 2
	sigaction(SIGQUIT, &sig_action, NULL); // 3
	sigaction(SIGKILL, &sig_action, NULL); // 9
	sigaction(SIGTERM, &sig_action, NULL); // 15
#endif

  int  i = 36;
	while (true)
	{
		if (sensorMode == CeleX5::Full_Picture_Mode)
		{
			if (!pCeleX5->getFullPicMat().empty())
			{
				cv::Mat fullPicMatMaster = pCeleX5->getFullPicMat(0);
				cv::Mat fullPicMatSlave = pCeleX5->getFullPicMat(1);

				cv::imshow(OPENCV_WINDOW_0, fullPicMatMaster);
				cv::waitKey(10);
				cv::imshow(OPENCV_WINDOW_1, fullPicMatSlave);

				cv::waitKey(10);

        char key_board = cv::waitKey(10);
			
        if (key_board == 's')
        {
		  std::cout << "截图" << std::endl;
          std::string FilenameMaster =  "/home/guoyeye/celex_ws/calibration/0_" + to_string(i) + ".jpg";
          std::string FilenameSlave =  "/home/guoyeye/celex_ws/calibration/1_" + to_string(i) + ".jpg";
          cv::imwrite(FilenameMaster, fullPicMatMaster);
          cv::imwrite(FilenameSlave, fullPicMatSlave);
          std::cout << "成功保存图片： 第" << i << "组" << std::endl;
          i++;
        }
        else if (key_board == 'q')
        {
          std::cout << "成功退出" << std::endl;
          break;
        }
			}
		}
		else if (sensorMode == CeleX5::Event_Address_Only_Mode)
		{
			if (!pCeleX5->getEventPicMat(CeleX5::EventBinaryPic).empty())
			{
				cv::Mat eventPicMatMaster = pCeleX5->getEventPicMat(CeleX5::EventBinaryPic,0);
				cv::Mat eventPicMatSlave = pCeleX5->getEventPicMat(CeleX5::EventBinaryPic,1);
				cv::imshow("Master-Event-EventBinaryPic", eventPicMatMaster);
				cv::imshow("Slave-Event-EventBinaryPic", eventPicMatSlave);
			}
			cv::waitKey(10);
		}
	}
	return 1;
}
