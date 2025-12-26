
#include "data_process/arc_tracker.h"

#include <iostream>
#include <vector>

int main() {
	//double config[10] = { 0, 10.0 };
	double firConfig[10] = { 1, 2, 3, 100 };
	filter_construct(0, firConfig, 5);

	std::vector<double> sample, ans;
	sample = { 289.998, 293.004, 293.004, 288.991, 297.993, 301, 310.002, 316.991, 325.002, 334.997,
		340.001, 336.996, 340.001, 331.991, 334.997, 334.997, 336.004, 331.991, 338.003, 334.005, 
		336.996, 336.996, 336.004, 334.005, 336.996, 336.004, 336.004, 340.001, 343.999, 349.996,
		349.996, 345.998, 346.99, 347.997, 351.003, 353.994, 357, 357, 345.998, 344.991, 
		343.999, 338.994, 334.997, 336.996, 338.003, 332.998, 336.004, 336.004, 323.995, 323.003, 
		332.998, 329.992, 321.996, 325.994, 330.999, 325.002, 329, 334.997, 336.004, 329,
		330.999, 323.003, 317.998, 321.996, 325.994, 327.993, 329, 330.999, 327.993, 331.991,
		332.998, 336.004, 342, 349.996, 349.996, 343.999, 342.992, 349.004, 340.993, 349.996,
		343.999, 336.004, 334.997, 332.998 };

	//for (size_t i = 0; i < 84; ++i) {
	//	ans.push_back(filter_process(0, sample[i]));
	//	std::cout << ans[i] << std::endl;
	//}

	//double config[10] = { 84, 75, 92 };
	//double ref = calc_interval_refrence(config, sample.data());
	//std::cout << ref << std::endl;

	double config[50], data[50];
	memset(config, 0, sizeof(double) * 50);
	config[0] = 1.0, config[2] = 0.280, config[3] = 3.000;
	config[7] = 1.0;
	config[29] = 0.0, config[31] = 180.0, config[32] = 180.0;
	config[33] = 0, config[34] = 10, config[35] = 0;
	config[36] = 10, config[37] = 50, config[38] = 20;
	config[40] = 0, config[41] = 0, config[42] = 180;
	config[47] = 62.832260, config[48] = 72.202260;
	calc_compensate(0, config, data);

	filter_deconstruct(0);
	return 0;
}
