
px4_add_module(
	MODULE modules__lidar
	MAIN lidar
	STACK 3000
	SRCS
		lidar.c
	DEPENDS
		platforms__common
	)
