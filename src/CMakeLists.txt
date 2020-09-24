#ADD_LIBRARY( myslam SHARED
#	./config/config.cpp
#	./frame/frame.cpp
#	./frame/calibrate_frame.cpp
#	./pose/pose.cpp
#	./pair/pair322.cpp
#	./camera/camera.cpp
#	./camera/calibrate_camera.cpp
#	./camera/fisheye_camera.cpp
#	./optimizer/pose_optimizer.cpp
#	./loader/loader.cpp
#	./system/system.cpp
#	./evaluator/evaluator.cpp
#	./evaluator/heuristic_evaluator.cpp
#)
#
#
#
#
#TARGET_LINK_LIBRARIES( myslam
#	${THIRD_PARTY_LIBS}
#)