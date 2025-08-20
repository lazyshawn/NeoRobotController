#include <windows.h>
#include<iostream>

//#include "Controller.h"
#include "SeamTracker.h"

std::shared_ptr<Controller> ZController(new Controller);
ScannerTracker tracker(ZController);

int main() {
	ZController->lazy_connect();
	//ZController.load_basic_project("D:\\CIMC\\FSAI_Teaching_Free_Weld_System\\zmotion_basic\\main.zar", 0);
	ScannerTracker tracker(ZController->get_handle());

	for (int i = 0; i < 1e5; ++i) {
		// ∂¡»°TCPª∫¥ÊŒª÷√
		std::vector<motion::Time_Pos> tcpBuffer;
		tracker.read_tcp_buffer(tcpBuffer);
		
		// ∑¢ÀÕ≤π≥•÷∏¡Ó
		tracker.send_tracking_cmd(i, { static_cast<float>(i+1), static_cast<float>(i + 2), static_cast<float>(i + 3) });
	}

	printf("Press <Enter> to exit.\n");
	getchar();

	return 0;
}
