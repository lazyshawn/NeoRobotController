
#include <iostream>
#include <fstream>
#include <chrono>

#include "interpolation/InterpSegment.h"
#include "interpolation/InterpDispatch.h"

int main() {
	InterpDispatcher dispatcher;
	InterpSignalIn signalIn;
	InterpSignalOut signalOut;
	DispatcherState dispatcherState;
	
	// 初始化
	for (int i=0; i< 1e3; ++i) {
		dispatcher.run_cycle_task(signalIn, signalOut, dispatcherState);
	}

	return 0;
}

