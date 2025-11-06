
#include "robot_interface/ZMotionController.h"

#include "RobotLogger.h"

#include <iostream>

namespace FSAIRobotInterface {

// 控制卡日志
log4cplus::Logger ControllerLog::logger;
ControllerLog::ControllerLog() {
	log4cplus::helpers::SharedObjectPtr<log4cplus::Appender> _append;
	_append = log4cplus::helpers::SharedObjectPtr<log4cplus::Appender>(new log4cplus::RollingFileAppender("./log/ZMotionController.log", 8 * 1024 * 1024, 8));//按照固定大小进行log分割
	_append->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::PatternLayout(LOG4CPLUS_TEXT("%D{%m/%d/%Y %H:%M:%S:%q} [%t] %-5p - %m %n"))));//("%D{%m/%d/%y %H:%M:%S},大写的D代表北京时间否则不准																															/* step 4: Instantiate a logger object */
	logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ZCONTROLLER_LOG"));
	logger.setLogLevel(log4cplus::INFO_LOG_LEVEL);
	logger.addAppender(_append);

	LOG4CPLUS_INFO(logger, "ZmotionController.");
}



int search_ethernet_list(std::vector<std::string>& ipList) {

	ipList.clear();

	//char buffer[100] = "127.0.0.1 196.168.1.1";
	char buffer[4096 + 256 + 1];
	int32 iresult;

	// 检索IP 地址到 Buffer
	iresult = ZAux_SearchEthlist(buffer, 4096 + 256 + 1, 100);
	// 失败退出 
	if (0 != iresult) {
		return iresult;
	}

	//从字符串转换过来 
	int ipos = 0;
	const char * pstring;
	pstring = buffer;
	char buffer2[256];
	buffer2[0] = '\0';
	for (int j = 0; j < 20; j++) {

		//跳过空格 
		while (' ' == pstring[0]) {
			pstring++;
		}
		memset(buffer2, 0, sizeof(buffer2));

		//获取地址到 buffer2
		ipos = sscanf(pstring, "%s", &buffer2);
		if (-1 == ipos) { break; }
		//printf("%s\n", buffer2);
		std::string ipStr = buffer2;
		ipList.push_back(ipStr);

		//指到IP 地址末尾
		while ((' ' != pstring[0]) && ('\t' != pstring[0]) && ('\0' != pstring[0])) {
			pstring++;
		}
	}

	return 0;
}

int search_ethernet(const char* ipAddress, uint32 uims) {
	return  ZAux_SearchEth(ipAddress, uims);
}

int get_max_pci_card() {
	return ZAux_GetMaxPciCards();
}


// 控制卡管理类
Controller::Controller() {

}

Controller::~Controller() {
	this->disconnect();
	handle = NULL;
}

int Controller::set_handle(const ZMC_HANDLE& handle_) {
	handle = handle_;
	return 0;
}

ZMC_HANDLE Controller::get_handle() {
	return handle;
}

/* ******************************** 控制卡连接 ********************************* */
int Controller::connect_eth(const char *ip_addr) {
	printf("Connecting to: %s... ", ip_addr);
	int ret = ZAux_OpenEth(const_cast<char *>(ip_addr), &handle);
	if (ERR_SUCCESS != ret) {
		printf("Failed!\n");
		handle = NULL;
		LOG4CPLUS_INFO(ControllerLog::getLogger(), cardName << " connected failed: " << ret);
		return errCodeBeg + 1;
	}
	printf("Succeed\n");

	// 日志模式
	ZAux_SetTraceFile(4, "sdf");

	// 控制卡名称
	cardName = std::string(ip_addr);

	return 0;
}

int Controller::connect_pci(uint32 cardNum, bool local, bool log) {
	printf("Connecting to pci: %d...", cardNum);

	std::string pciStr = local ? "LOCAL" : "PCI" + std::to_string(cardNum);

	// 控制卡名称
	cardName = pciStr;

	int ret = ZAux_FastOpen(local ? 5 : 4, const_cast<char*>(pciStr.c_str()), 1000, &handle);
	if (ret != 0) {
		LOG4CPLUS_INFO(ControllerLog::getLogger(), cardName << ": PCI fast open failed.");

		ret = ZAux_OpenPci(cardNum, &handle);
		if (ret != 0) {
			printf("Failed!\n");
			LOG4CPLUS_INFO(ControllerLog::getLogger(), cardName << "PCI open failed.");
		}
	}

	if (log) {
		ZAux_SetTraceFile(4, "sdf");
	}


	return ret;
}

int Controller::lazy_connect() {
	// 750
	if (this->connect_pci(0, true, true) == 0) {
		return 0;
	}
	// 仿真器
	else if (this->connect_eth((char *)"127.0.0.1") == 0) {
		return 0;
	}
	// 默认实体控制卡 IP
	else if (this->connect_eth((char *)"192.168.1.14") == 0) {
		return 0;
	}
	// 协作臂 IP
	else if (this->connect_eth((char *)"169.254.180.11") == 0) {
		return 0;
	}
	// PCI 卡
	else if (this->connect_pci(0) == 0) {
		return 0;
	}
	// 连接失败
	return errCodeBeg + 1;
}

int Controller::connect(const std::string& addr) {
	if (addr.size() > 2) {
		return connect_eth(addr.c_str());
	}
	else {
		// 转换成uint32
		uint32 cardNum = static_cast<uint32>(std::atoi(addr.c_str()));
		return connect_pci(cardNum);
	}
	return 0;
}

int Controller::disconnect() {
	//关闭连接 
	if (ZAux_Close(handle) > 0) {
		printf("Error: # ZauxRobot::disconnect()!\n");
		return 1;
	}
	printf("connection closed!\n");
	handle = NULL;
	return 0;
}


/* ******************************** 控制卡设置 ********************************* */
int Controller::load_basic_pragma(const char *basPath, uint32_t mode) {
	// 加载 bas 程序
	if (ZAux_BasDown(handle, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return errCodeBeg + 2;
	}
	return 0;
}

int Controller::load_basic_project(const char *basPath, uint32_t mode) {
	// 加载 zar 程序
	if (ZAux_ZarDown(handle, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return errCodeBeg + 2;
	}
	return 0;
}

int Controller::allocate_robot_id() {
	int robotID = -1;

	// 分配机器人ID
	uint8_t robotList = robotHandle;
	for (size_t i = 0; i < 8; ++i) {
		if (robotList % 2 == 0) {
			robotID = i;
			break;
		}
		robotList = robotList >> 1;
	}

	// 分配机器人轴号

	return robotID;
}

int Controller::add_robot(int id) {
	// robotId 合法性检查
	if (id >= 8 || id < 0) {
		return -1;
	}

	// robotId 已被占用，使用 allocate_robot_id 申请新robotId
	uint8_t robotList = robotHandle;
	if ((robotList >> id) % 2 == 1) {
		return -2;
	}

	// 将robotId标记为已占用
	robotHandle += (1 << id);

	return 0;
}

int Controller::remove_robot(int id) {
	// robotId 合法性检查
	if (id >= 8 || id < 0) {
		return 1;
	}

	// 指定 robotId 未被占用
	uint8_t robotList = robotHandle;
	if ((robotList >> id) % 2 == 0) {
		return 2;
	}

	// 将 robotId 标记为未占用
	robotHandle -= (1 << id);

	return 0;
}


/* ******************************** 异常处理 ********************************* */
int Controller::handle_zaux_error(int32 errCode) {
	if (errCode != 0) {
	}
	return errCode;
}


/* ******************************** 读写轴参数 ********************************* */
// 加载配置文件
int Controller::load_config(const std::string& fname) {
	std::ifstream file;
	file.open(fname, std::ios::in);
	// 文件打开失败
	if (!file.is_open()) {
		std::cout << "not found" << std::endl;
		return -1;
	}

	char cmdbuffAck[2048];
	std::string buf;
	while (getline(file, buf)){
		// 跳过空行
		if (std::all_of(buf.begin(), buf.end(), isspace))
			continue;

		// 跳过备注
		if (buf[0] == '\'')
			continue;

		// 下发指令
		int ret = sendCmd(buf.c_str(), cmdbuffAck);

		// 指令下发失败
		if (ret != 0) {
			file.close();
			return -2;
		}
	}

	file.close();

	return 0;
}

int Controller::export_config(const std::string& fname) {
	std::ofstream file;
	file.open(fname, std::ios::out);
	// 文件打开失败
	if (!file.is_open()) {
		std::cout << "not found" << std::endl;
		return -1;
	}
	file.setf(std::ios::fixed);
	file.precision(5);
	//file.unsetf(std::ios::showpoint);

	//! 所有VR: ?SYS_ZFEATURE(16)
	// 一次读一千个
	size_t unitSize = 1000;

	// 只读四个机器人配置和全局配置
	for (size_t i = 0; i < 5; ++i) {
		std::vector<float> pfValue(unitSize, 0);

		int ret = ZAux_Direct_GetVrf(handle, i*unitSize, unitSize, pfValue.data());

		if (ret != 0) {
			file.close();
			return -1;
		}
		else {
			// 保存非零值
			for (size_t j = 0; j < unitSize; ++j) {
				if (std::fabs(pfValue[j]) > 1e-2) {
					//std::cout << "VR(" << i * unitSize + j << ")=" << pfValue[j] << std::endl;
					file << "VR(" << i * unitSize + j << ")=" << pfValue[j] << std::endl;
				}
			}
		}
	}

	file.close();
	return 0;
}


// 获取轴号信息
int Controller::get_axis_param(const std::vector<int>& axisList, const char* paramName, std::vector<float>& paramList) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	if (axisList.size() < 1) {
		return 1;
	}
	paramList = std::vector<float>(axisList.size(), 0);

	// 生成命令
	sprintf(cmdbuff, "?%s(%d)", paramName, axisList[0]);
	for (size_t i = 1; i < axisList.size(); ++i) {
		sprintf(tempbuff, ",%s(%d)", paramName, axisList[i]);
		strcat(cmdbuff, tempbuff);
	}

	int32 iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);

	// 判断返回状态
	if (ERR_OK != iresult || 0 == strlen(cmdbuffAck)) {
		return errCodeBeg + 4;
	}

	// 解析返回值
	std::stringstream ackStr(cmdbuffAck);
	std::string word;
	// Extract word from the stream
	size_t i = 0;
	while (ackStr >> word) {
		if (i >= axisList.size())
			break;
		//paramList.push_back(std::stof(word));
		paramList[i++] = std::stof(word);
	}

	return 0;
}


int Controller::get_axis_param(int axis, const char* paramName, float& value) {
	std::vector<int> axisList = { axis };
	std::vector<float> paramList = { value };

	int ret = get_axis_param(axisList, paramName, paramList);
	value = paramList[0];

	return ret;
}

// 设置轴号信息
int Controller::set_axis_param(const std::vector<int>& axisList, const char* paramName, const std::vector<float>& paramList, int principal) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	if (axisList.size() < 1/* || paramList.size() < axisList.size()*/) {
		return 1;
	}

	int num = (std::min)(paramList.size(), axisList.size());

	// 立即设置
	if (principal < 0) {
		sprintf(cmdbuff, "%s(%d)=%f", paramName, axisList[0], paramList[0]);
		for (size_t i = 1; i < num; ++i) {
			sprintf(tempbuff, "\n%s(%d)=%f", paramName, axisList[i], paramList[i]);
			strcat(cmdbuff, tempbuff);
		}
	}
	// 缓冲中修改TABLE
	else if (_stricmp(paramName, "TABLE") == 0) {
		sprintf(cmdbuff, "MOVE_TABLE(%d,%f) axis(%d)", axisList[0], paramList[0], principal);
		for (size_t i = 1; i < num; ++i) {
			sprintf(tempbuff, "\nMOVE_TABLE(%d,%f) axis(%d)", axisList[i], paramList[i], principal);
			strcat(cmdbuff, tempbuff);
		}
	}
	// 缓冲中设置
	else {
		sprintf(cmdbuff, "MOVE_PARA(%s,%d,%f) axis(%d)", paramName, axisList[0], paramList[0], principal);
		for (size_t i = 1; i < num; ++i) {
			sprintf(tempbuff, "\nMOVE_PARA(%s,%d,%f) axis(%d)", paramName, axisList[i], paramList[i], principal);
			strcat(cmdbuff, tempbuff);
		}
	}
	//std::cout << cmdbuff << std::endl;
	ret = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}

int Controller::set_axis_param(int axis, const char* paramName, float value, int principal) {
	std::vector<int> axisList = { axis };
	std::vector<float> paramList = { value };

	int ret = set_axis_param(axisList, paramName, paramList, principal);

	return ret;
}

int Controller::set_base_param(int axis, const char* paramName, const std::vector<float>& value) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	sprintf(cmdbuff, "BASE(%d)\n%s(", axis, paramName);
	for (size_t i = 0; i < value.size()-1; ++i) {
		sprintf(tempbuff, "%f,", value[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", value.back());
	strcat(cmdbuff, tempbuff);

	ret = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);

}

int Controller::set_axis_connect(const std::vector<int>& master, const std::vector<int>& slave, const std::vector<float>& ratio) {

	size_t num = std::min(master.size(), slave.size());
	num = num < ratio.size() ? num : ratio.size();

	for (size_t i = 0; i < num; ++i) {
		int ret = ZAux_Direct_Connect(handle, ratio[i], master[i], slave[i]);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}


int Controller::addax(const std::vector<int>& axis, const std::vector<int>& addAxis) {
	int ret = 0;
	int num = (std::min)(axis.size(), addAxis.size());

	for (size_t i = 0; i < num; ++i) {
		ret = ZAux_Direct_Single_Addax(handle, axis[i], addAxis[i]);
	}
	
	return ret;
}

int Controller::get_register(int start, int numes, std::vector<float>& pfValue, int type) {
	pfValue = std::vector<float>(numes, 0);

	int ret;
	if (type == 0) {
		ret = ZAux_Direct_GetTable(handle, start, numes, pfValue.data());
	}
	else if (type == 1) {
		ret = ZAux_Direct_GetVrf(handle, start, numes, pfValue.data());
	}
	else {
		return -1;
	}

	return 0;
}

int Controller::set_register(int start, const std::vector<float>& pfValue, int type) {
	if (type == 0) {
		ZAux_Direct_SetTable(handle, start, pfValue.size(), const_cast<float*>(pfValue.data()));
	}
	else if (type == 1) {
		ZAux_Direct_SetVrf(handle, start, pfValue.size(), const_cast<float*>(pfValue.data()));
	}
	else {
		return -1;
	}

	return 0;
}

long long Controller::get_time_stamp(int taskId) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	//生成命令
	sprintf(cmdbuff, "?ticks(%d)", taskId);

	int32 iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);

	// 判断返回状态
	if (ERR_OK != iresult) {
		return handle_zaux_error(iresult);
	}
	if (0 == strlen(cmdbuffAck)) {
		return handle_zaux_error(ERR_NOACK);
	}

	// 解析返回值
	std::stringstream ackStr(cmdbuffAck);
	std::string word;
	// Extract word from the stream
	ackStr >> word;
	return std::stoll(word);
}

// 保存table
int Controller::save_table(size_t startIdx, size_t num, const std::string& path) {
	if (num == 0) {
		return 0;
	}
	int ret = 0;

	// 单次最大读取数量
	size_t maxNum = 1000;
	// 读取缓冲
	std::vector<float> tableData(maxNum, 0);
	// 读取次数
	size_t times = std::floor(num / maxNum);
	// 输出文件
	std::ofstream out(path, std::ios::trunc);

	for (size_t i = 0; i < times; ++i) {
		// 读取数据
		ret = ZAux_Direct_GetTable(handle, startIdx + maxNum * i, maxNum, tableData.data());
		// 判断返回状态
		if (ret != 0)
			return handle_zaux_error(ret);
		// 保存到文件
		for (const auto& data : tableData) {
			out << data << std::endl;
		}
	}

	// 剩余数据长度
	size_t remain = num - maxNum * times;
	// 读取剩余数据
	ZAux_Direct_GetTable(handle, startIdx + maxNum * times, remain, tableData.data());
	for (size_t i = 0; i < remain; ++i) {
		out << tableData[i] << std::endl;
	}

	return handle_zaux_error(ret);
}

int Controller::move_wait(uint32 base_axis, const char * paraname, int inum, int Cmp_mode, float fvalue) {
	return ZAux_Direct_MoveWait(handle, base_axis, const_cast<char *>(paraname), inum, Cmp_mode, fvalue);
}

int Controller::get_in(int ioNum) {
	uint32 pValue;

	ZAux_Direct_GetIn(handle, ioNum, &pValue);

	return pValue;
}
int Controller::get_invert_in(int ioNum) {
	int pValue;

	ZAux_Direct_GetInvertIn(handle, ioNum, &pValue);

	return pValue;
}
int Controller::set_invert_in(int ioNum, int bIfInvert) {

	int ret = ZAux_Direct_SetInvertIn(handle, ioNum, bIfInvert);

	return ret;
}

int Controller::get_op(int ioNum) {
	uint32 pValue;

	ZAux_Direct_GetOp(handle, ioNum, &pValue);

	return pValue;
}
int Controller::set_op(int ioNum, int opState) {

	int ret = ZAux_Direct_SetOp(handle, ioNum, opState);

	return ret;
}

int Controller::get_node_info(std::vector<std::vector<int>>& info) {

	int ret = 0;
	std::vector<int> infoIdx = { 0,1,2,3 };

	// 获取节点数
	std::vector<float> value;
	ret = get_axis_param({ 0 }, "NODE_COUNT", value);
	int num = static_cast<int>(value[0]);

	// 节点数异常
	if (num <= 0)
		return -1;

	info = std::vector<std::vector<int>>(num);

	for (size_t i = 0; i < num; ++i) {
		char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

		auto paramList = std::vector<float>(infoIdx.size(), 0);

		// 生成命令
		sprintf(cmdbuff, "?NODE_INFO(0,%d,%d)", num, infoIdx[0]);
		for (size_t j = 1; j < infoIdx.size(); ++j) {
			sprintf(tempbuff, ",NODE_INFO(0,%d,%d)", num, infoIdx[j]);
			strcat(cmdbuff, tempbuff);
		}

		int32 iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);

		// 判断返回状态
		if (ERR_OK != iresult || 0 == strlen(cmdbuffAck)) {
			return errCodeBeg + 4;
		}

		// 解析返回值
		std::stringstream ackStr(cmdbuffAck);
		std::string word;
		// Extract word from the stream
		size_t j = 0;
		while (ackStr >> word) {
			if (j >= num)
				break;
			//paramList.push_back(std::stof(word));
			paramList[j++] = std::stof(word);
		}

		for (size_t j = 0; j < paramList.size(); ++j) {
			info[i].push_back(static_cast<int>(paramList[j]));
		}
	}
	
	return 0;

}

int Controller::write_node_pdo(int node, int index, int subIndex, int type, int value) {

	int ret = ZAux_BusCmd_NodePdoWrite(handle, node, index, subIndex, type, value);
	return ret;

}

int Controller::read_node_pdo(int node, int index, int subIndex, int type, int *value) {

	int ret = ZAux_BusCmd_NodePdoRead(handle, node, index, subIndex, type, value);
	return ret;

}


/* ******************************** 基础运动指令封装 ********************************* */
// jog
int Controller::axis_jog(int axis, int dir) {
	int ret;
	// 运动结束
	if (dir == 0) {
		ret = ZAux_Direct_Single_Cancel(handle, axis, 4);
	}
	// 正向运动
	else if (dir > 0) {
		ret = ZAux_Direct_Single_Vmove(handle, axis, 1);
	}
	// 负向运动
	else {
		ret = ZAux_Direct_Single_Vmove(handle, axis, -1);
	}

	return ret;
}

// move
int Controller::move(const std::vector<int>& axis, const std::vector<float>& relMove, int moveType, const std::vector<int>& mask) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(relMove.size(), axis.size());
	if (num < 1)
		return 1;

	int ret = 0;
	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048] = { 0 };

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num; i++) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}

		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")\n");

	if (moveType == 0) {
		strcat(cmdbuff, "MOVESP(");
	}
	else if (moveType == 1) {
		strcat(cmdbuff, "MOVE(");
	}

	for (size_t i = 0; i < num; i++) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}
		sprintf(tempbuff, "%f,", relMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")");

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ret;
}

// moveABS
int Controller::moveABS(const std::vector<int>& axis, const std::vector<float>& endMove, int moveType, const std::vector<int>& mask) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(endMove.size(), axis.size());
	if (num < 1)
		return 1;

	int ret = 0;
	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num; i++) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}

		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")\n");

	if (moveType == 0) {
		strcat(cmdbuff, "MOVEABSSP(");
	}
	else if (moveType == 1) {
		strcat(cmdbuff, "MOVEABS(");
	}

	for (size_t i = 0; i < num ; i++) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}

		sprintf(tempbuff, "%f,", endMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")");

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ret;
}

//
int Controller::baseCMD(const std::vector<int>& axis, const char * paraname, const std::vector<float>& cmdData) {

	int ret = 0;
	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048] = {0};

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < axis.size(); i++) {
		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")\n");

	strcat(cmdbuff, paraname);
	strcat(cmdbuff, "(");
	for (size_t i = 0; i < cmdData.size(); i++) {
		sprintf(tempbuff, "%f,", cmdData[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")");

	//调用命令执行函数
	ret = sendCmd(cmdbuff, cmdbuffAck);
	return ret;

}

int Controller::axis_stop(const std::vector<int>& axis, int mode) {

	int ret = 0;
	for (size_t i = 0; i < axis.size(); ++i) {
		int temp = ZAux_Direct_Single_Cancel(handle, axis[i], mode);
		if (temp != 0) {
			ret = temp;
		}
	}
	return ret;

}

int Controller::sendCmd(const char* pszCommand, char* psResponse, int cmdType) {
	int ret = 0;

	if (cmdType == 0) {
		ret = ZAux_Execute(handle, pszCommand, psResponse, 2048);
	}
	else {
		ret = ZAux_DirectCommand(handle, pszCommand, psResponse, 2048);
	}

	return ret;
}


int Controller::read_message() {

	// 读取缓冲
	char psResponse[2048];
	// 实际读取长度
	uint32 puiread;

	int ret = ZMC_ReadMessage(handle, psResponse, 2048, &puiread);

	if (ret == 0 && puiread > 0) {
		//printf("%.*s", puiread, psResponse);
		char res[2048];
		sprintf(res, "%.*s", puiread, psResponse);
		LOG4CPLUS_INFO(ControllerLog::getLogger(), cardName << ": " << res);
	}

	return 0;

}


} // namespace ZMotionRobot