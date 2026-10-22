px4_add_module(
	MODULE modules__capable_control
	MAIN capable_control
	SRCS
		CapableControl.cpp
		CapableControl.hpp
		CapableFlightController.hpp
		Scenarios.hpp
	DEPENDS
		px4_work_queue
	)
