#include <iostream>

# include "robot_webservice/webservice.h"

int main() {
	std::cout << "Hello world." << std::endl;

	WebService client;
	std::string msg;
	client.get_abb("/", msg);
	//msg = client.get_error_code();

	std::cout << msg << std::endl;

	return 0;
}
