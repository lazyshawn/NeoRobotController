
#ifndef CPPHTTPLIB_OPENSSL_SUPPORT
#define CPPHTTPLIB_OPENSSL_SUPPORT
#endif

#include <httplib.h>
#include <Windows.h>
#include <memory>
#include <nlohmann/json.hpp>

#include "robot_webservice/webservice.h"

std::shared_ptr<httplib::Client> client;

WebService::WebService() {

	client = std::make_shared<httplib::Client>("http://192.168.125.1");
	client->set_digest_auth("Default User", "robotics");

}


std::string WebService::get_error_code() {
	std::string msg = "hello world";
	httplib::Params Params = {
	  { "json", "1" }
	};

	auto res = client->Get("/rw", Params, httplib::Headers{});
	auto status = res->status;

	msg = res->body;
	
	return msg;
}


std::string WebService::get_error_code_ssl() {
	std::string msg = "hello world";
	
	return msg;
}


int WebService::get_abb(const std::string& request, std::string& response) {

	std::string msg = "hello world";
	httplib::Params Params = {
	  { "json", "1" }
	};

	auto res = client->Get(request, Params, httplib::Headers{});
	auto status = res->status;

	nlohmann::json ans = nlohmann::json::parse(res->body);
	response = res->body;

	return res->status;

}
