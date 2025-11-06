#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "zmotion.h"
#include "zauxdll2.h"


namespace FSAIRobotInterface {

/**
* @brief  搜索当前网段下的 IP 地址
* @param  ipList    IP 地址列表
*/
int search_ethernet_list(std::vector<std::string>& ipList);

/**
* @brief  搜索当前网段下的 IP 地址
* @param  ipAddress    IP 地址
* @param  uims         超时时间
* @return 存在返回 0，非 0 详见错误码说明
*/
int search_ethernet(const char* ipAddress, uint32 uims = 1000);

/**
* @brief  搜索当前最大 PCI 卡数
* @return 最大 PCI 卡数
*/
int get_max_pci_card();

class Controller {
	//! 异常码
	const static int errCodeBeg = 10000;
	//! 控制卡数量
	static int numOfController;
	//! 控制器句柄
	ZMC_HANDLE handle = NULL;
	//! 已连接机器人
	uint8_t robotHandle = 0;
	//！ 控制卡状态
	uint64_t controllerStatus = 0;
	//！控制卡名称
	std::string cardName;

	// 互斥锁与条件变量 
	//std::mutex mtx;
	//std::condition_variable cv;
	//bool taskDone = false;

public:
	Controller();
	~Controller();

	/**
	* @brief 设定控制卡句柄
	* @param handle    控制卡句柄
	*/
	int set_handle(const ZMC_HANDLE& handle_);
	/**
	* @brief 获取控制卡句柄
	* @return    控制卡句柄
	*/
	ZMC_HANDLE get_handle();

	/**
	* @brief 连接控制器
	*/
	int connect_eth(const char *ip_addr);
	/**
	* @brief  连接控制器
	* @param  cardNum    卡号
	* @param  local      T - 750, F - 464
	* @param  log        T - 开启, F - 关闭
	*/
	int connect_pci(uint32 cardNum, bool local = false, bool log = false);
	int connect(const std::string& addr);
	int lazy_connect();
	/**
	* @brief 断开连接
	*/
	int disconnect();
	/**
	* @brief 控制卡重启
	*/
	int controller_reboot();

	/**
	* @brief 保存设备信息
	*
	* 如控制卡型号，固件版本，节点数等
	*/
	int save_device_info();
	/**
	* @brief load_basic_project
	*        烧录 basic 工程到控制器
	* @param basPath    basic 程序路径
	* @param mode       烧录模式；
			 0          烧录到 RAM
			 1          烧录到 ROM
	*/
	int load_basic_project(const char *basPath, uint32_t mode = 0);
	int load_basic_pragma(const char *basPath, uint32_t mode = 0);

	/**
	* @brief allocate_robot_id
	* @return   从小到大分配可用的机器人ID
	*/
	int allocate_robot_id();
	/**
	* @brief add_robot
	* @param   robotId    申请新增的机器人robotId
	*/
	int add_robot(int id);
	/**
	* @brief remove_robot
	* @param   robotId    申请移除的机器人robotId
	*/
	int remove_robot(int id);
	/**
	* @brief get_robot_id
	* @return   已经被占用的机器人ID
	*/
	std::vector<int> get_robot_id();

	/**
	* @brief 异常处理
	* @param    errCode    错误码
	*/
	//! 错误码处理
	int32 handle_zaux_error(int32 errCode);
	////! 上位机紧急停止
	//int32 emergency_stop();
	////! 上位机紧急暂停
	//int32 emergency_pause();
	////! 上位机紧急恢复
	//int32 emergency_resume();
	////! 控制卡标志位复位
	//int32 basic_reset();

	/**
	* @brief  获取下位机时间戳
	* @param       taskId     任务号
	* @return      下位机时间戳，即>> ?ticks(taskId) 的返回值
	*/
	long long get_time_stamp(int taskId = 0);

	/**
	* @brief  加载配置文件
	* @param       fname     配置文件路径
	*/
	int load_config(const std::string& fname);

	/**
	* @brief  导出配置文件
	* @param       fname     配置文件路径
	*/
	int export_config(const std::string& fname);

	/**
	* @brief  读取多轴参数
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param[out]  paramList    返回的参数列表
	*/
	int get_axis_param(const std::vector<int>& axisList, const char* paramName, std::vector<float>& paramList);
	int get_axis_param(int axis, const char* paramName, float& value);

	/**
	* @brief 设置多轴参数
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param       paramList    参数数组
	* @param       principal    主轴索引: -1 立即设置; >0 缓冲中设置
	*/
	int set_axis_param(const std::vector<int>& axisList, const char* paramName, const std::vector<float>& paramList, int principal = -1);
	int set_axis_param(int axis, const char* paramName, float value, int principal = -1);
	int set_base_param(int axis, const char* paramName, const std::vector<float>& value);

	int set_axis_connect(const std::vector<int>& master, const std::vector<int>& slave, const std::vector<float>& ratio);

	/**
	* @brief 轴叠加
	* @param axis      被叠加轴
	* @param addAxis   叠加轴
	*/
	int addax(const std::vector<int>& axis, const std::vector<int>& addAxis);

	/**
	* @brief 读取寄存器值
	* @param       start      操作的寄存器起始编号
	* @param       numes      操作个数
	* @param[out]  pfValue    数据列表
	* @param       type       寄存器类型: 0 TABLE; 1 VR
	*/
	int get_register(int start, int numes, std::vector<float>& pfValue, int type = 0);

	/**
	* @brief 设置寄存器值
	* @param       start      操作的寄存器起始编号
	* @param       numes      操作个数
	* @param       pfValue    数据列表
	* @param       type       寄存器类型: 0 TABLE; 1 VR
	*/
	int set_register(int start, const std::vector<float>& pfValue, int type = 0);

	/**
	* @brief 保存table数据到本地
	*/
	int save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	/**
	* @brief  缓冲等待
	* @param  base_axis	插补主轴编号
	* @param  paraname	参数名字符串 DPOS MPOS IN AIN VPSPEED MSPEED MODBUS_REG MODBUS_IEEE MODBUS_BIT NVRAM VECT_BUFFED  REMAIN
	* @param  inum		参数编号或轴号
	* @param  Cmp_mode	比较条件 1 >=   0=  -1<=  对IN等BIT类型参数无效。
	* @param  fvalue	修改值
	*/
	int move_wait(uint32 base_axis, const char * paraname, int inum, int Cmp_mode, float fvalue);

	/**
	* @brief 设置IO
	*/
	int get_in(int ioNum);
	int get_invert_in(int ioNum);
	int set_invert_in(int ioNum, int bIfInvert);
	int get_op(int ioNum);
	int set_op(int ioNum, int state);

	/**
	* @brief  获取总线节点信息
	* @param  info    节点信息 <VENDER, DEVICE, VERSION, ALIAS>
	* @return 0       正常返回
	          -1      节点数异常
	*/
	int get_node_info(std::vector<std::vector<int>>& info);

	/**
	* @brief  修改 PDO
	* @param  node        节点序号
	* @param  index       PDO 索引
	* @param  subIndex    PDO 子索引
	* @param  type        PDO 类型
	* @param  value       待写入值
	*/
	int write_node_pdo(int node, int index, int subIndex, int type, int value);
	/**
	* @brief 读取 PDO
	* @param        node        节点序号
	* @param        index       PDO 索引
	* @param        subIndex    PDO 子索引
	* @param        type        PDO 类型
	* @param [out]  value       读取值
	*/
	int read_node_pdo(int node, int index, int subIndex, int type, int *value);




	/**
	* @brief Jog 点动
	* @param       axis    运动轴号
	* @param       dir     运动方向
					 0     停止运动
				     1     正向运动
				    -1     负向运动
	*/
	int axis_jog(int axis, int dir);

	/**
	* @brief 多轴相对运动
	* @param       axis         运动轴号
	* @param       relMove      相对位移
	* @param       moveType     运动类型
	                       0    SP 运动
						   1    普通运动
	* @param       mask         轴屏蔽状态
	*/
	int move(const std::vector<int>& axis, const std::vector<float>& relMove, int moveType = 0, const std::vector<int>& mask = {});
	/**
	* @brief 多轴绝对运动
	* @param       axis         运动轴号
	* @param       endMove      目标位置
	* @param       moveType     运动类型
						   0    SP 运动
						   1    普通运动
	* @param       mask         轴屏蔽状态
	*/
	int moveABS(const std::vector<int>& axis, const std::vector<float>& endMove, int moveType = 0, const std::vector<int>& mask = {});

	int baseCMD(const std::vector<int>& axis, const char * paraname, const std::vector<float>& cmdData);

	/**
	* @brief 轴停止
	* @param    axis    停止轴号
	* @param    mode    停止模式
	*/
	int axis_stop(const std::vector<int>& axis, int mode = 2);

	/**
	* @brief 下发自编指令
	* @param       pszCommand     下发指令，长度2048
	* @param       psResponse     指令返回值，长度2048
	* @param       cmdType        0 - ZAux_Execute. 1 - ZAux_DirectCommand
	*/
	int sendCmd(const char* pszCommand, char* psResponse, int cmdType = 1);
	/**
	* @brief 捕获Rtsys日志
	*/
	int read_message();
};

} // namespace ZMotionRobot