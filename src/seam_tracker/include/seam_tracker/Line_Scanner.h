#pragma once

#include "Data_Info.h"
#include <eigen3/Eigen/Dense>
#include <array>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <chrono>

/*
//线扫扫描器：实现扫描获取点云
//存在独立线程：1、从下位机获取位姿时序，2、根据时序，获得上位机中的对应点云
//功能：
//1、获取下位机位姿时序并存储；
//2、获取上位机点云；
//3、计算对应点云时间戳的相机位姿
//4、将匹配的位姿+点云，输出给相机上位机(tracking & photo)；
//5、告知相机，拍照完毕;

//存在两个线程，一个与下位机交互读取点云；一个与相机上位机比对点云+位姿插值；
//读取下位机位姿要求：一个线扫对应table表中要维护两个数据结构，每个数据结构有个完成标志1及
//下层写入时，到最后一位后，将另一个数据的可读改为false，马上将该数据中的可读标志改为true
*/
namespace motion
{
	struct Time_Stamp
	{
		uint16_t ms;//millisecond
		uint16_t s; //second
		uint16_t mi;//minute
		uint16_t h; //hour
		uint16_t d; //day
		uint16_t m; //month
		uint16_t y; //year
		//this比t更后，return：true=this比t更后，false=t比this更后
		bool is_later(const Time_Stamp& t)const;
		bool is_same(const Time_Stamp& t)const;
		//计算两个时间的差值，只区分秒以下，单位：ms
		int decl_ms(const Time_Stamp& t)const;
		std::chrono::system_clock::time_point make_time()const;
		std::tm make_tm()const;
	};

	struct Time_Pos
	{
		Time_Stamp time;
		Cart_Pnt tcp_pos;
	};

	enum class Scan_Mode
	{//线扫模式：用于区分处理后的位姿+点云的出口（相机 or 跟踪器）
		Photo = 0, //单纯拍照
		Traker  //线扫跟踪
	};

	struct Pos_List
	{//内存储10个点位，写线程=5ms，读线程=10ms，若大于相机频率也可以，因为有位姿插值
		uint8_t done = 0;//0=无效(默认):数据写入中置位0；1=有效：写入完毕置位1；2：数据读取中置位2，此时不能写入数据
		uint8_t read = false;//true=can read
		uint8_t write = true;//true=can wirte or false=can not write
	};
	/*
	//有P1,P2两个Pos_List数据结构，读线程t1,写线程t2；有公共tag：1=P1可读；2=P2可读；0=均不可读，均可写
	//初始化P1/P2位可写，不可读；
	//写线程必须根据tag+1，检测Pi为可写后，将Pi置位不可读，写完后，对Pi设为可读+将tag置位i，且若Pi+1为可写则将其设为不可读
	//读线程根据tag检测Pi为可读后，将Pi置位不可写，读完后，对Pi设为可写；
	//写线程必须根据tag+1,写入Pi数据，否则可能导致读取位姿有往返；
	*/

	class Line_Scanner
	{
	public:
		Line_Scanner(const std::array<int, 4>& ip);
		//设置手眼矩阵
		void set_heye(const Cart_Pnt& heye);
		//设置相机ip
		void set_camera_ip(const std::array<int, 4>& ip);
		//开启线扫,start:true=启动，false=关闭,return：true=线扫成功
		bool set_start(bool start);
		//获取当前线扫状态是否开启：true=开启
		bool get_start()const;
		//关闭相机
		void close_camera();
		//开启相机
		void open_camera();
		//检测相机是否online
		bool is_online()const;
		//链接相机
		bool set_connect();
		//由杨旭运动层调用，设置相机上位机线扫完成状态，以告知调度层拍照完成
		void set_scan_done();
		//比对两个pos是否相同,exp=检测精度
		static bool same_pos(const Cart_Pnt& p1, const Cart_Pnt& p2, double exp = 0.01);
		//将时间戳合（年+月+日+时）并为一个数据
		static int year_hour(int y, int m, int d, int h);
		//将时间戳（分+秒+毫秒），合并为一个数据
		static int minute_ms(int mi, int s, int ms);
		static void ask_year_hour(const int yh, int& y, int& m, int& d, int& h);
		static void ask_mms(const int mms, int minute, int s, int ms);
	private:
		//独立线程运行线扫，读取下位机位姿队列，匹配上位机点云
		void run_scan();
		//由run_scan触发，根据下位机位姿对应相机点云时间戳，根据线扫类型压入相机or跟踪器队列
		void post_product();
		Eigen::Matrix4f ask_eye_matrix(const Cart_Pnt& p)const;

	private:
		std::atomic_bool start_ = false;//机器人到位后开启线扫标志位
		Scan_Mode scan_type_ = Scan_Mode::Photo;//默认线扫拍照
		std::mutex mtx_;
		std::condition_variable cdv_;//用于唤醒读取位姿线程
		std::mutex post_mtx_;
		std::condition_variable post_cdv_;//后处理线程（由线扫线程触发，）
		std::mutex read_mtx_;//读取数据锁
		
		int pose_num_ = 1000;//poses_最大数量，避免内存占用过大
		Cart_Pnt hand_eye_;//手眼矩阵
		Eigen::Matrix4f eye_mtx_;//手眼矩阵
		std::array<int, 4> camera_ip_;//相机ip
		std::deque<Time_Pos> poses_;  //位姿时序
	};
}
