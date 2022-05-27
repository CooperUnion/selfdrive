/*
 * fittingApproach.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FITTINGAPPROACH_H_
#define FITTINGAPPROACH_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class FittingApproach {
public:
        FittingApproach(){
        };
        void fitting(cv::Mat& mat, cv::Rect& box, std::vector<cv::Point>& splinePoints);
};

#endif /* FITTINGAPPROACH_H_ */
