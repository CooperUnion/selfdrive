#pragma once
#include <lane_detector/lane_tracker/defines.h>
#include <opencv2/opencv.hpp>

// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
//die folgenden Codeabschnitte sind open source und wurden aus https://github.com/Smorodov/Multitarget-tracker genommen
class TKalmanFilter
{
public:
	cv::KalmanFilter* kalman;
	track_t deltatime;
	Point_t LastResult;
	TKalmanFilter(Point_t p, track_t dt = 0.2, track_t Accel_noise_mag = 0.5);
	~TKalmanFilter();
	Point_t GetPrediction();
	Point_t Update(Point_t p, bool DataCorrect);
};
