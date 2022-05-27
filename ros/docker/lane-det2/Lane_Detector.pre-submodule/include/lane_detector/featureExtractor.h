/*
 * FeatureExtractor.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <lane_detector/DetectorConfig.h>
#include <cv.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/utils.h>

class FeatureExtractor {
public:
        FeatureExtractor(){
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
        };
        void extract(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& features);
private:
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
};

#endif /* FEATUREEXTRACTOR_H_ */
