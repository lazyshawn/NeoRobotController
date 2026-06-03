#pragma once

#include <string>
//#include <restbed>

#include "common/ExportSharedAPI.h"

class SHARE_API_ WebService {

public:
	WebService();

	std::string get_error_code();
	std::string get_error_code_ssl();

	int get_abb(const std::string& request, std::string& response);

};
