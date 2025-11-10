#pragma once

#include <string>
//#include <restbed>

class WebService {

public:
	WebService();

	std::string get_error_code();
	std::string get_error_code_ssl();

	int get_abb(const std::string& request, std::string& response);

};
