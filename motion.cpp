
#include "motion.h"

void distMapBGR(const cv::Mat &a, const cv::Mat &b, cv::Mat &out){
	// #pragma omp parallel
	for(int y = 0; y < a.rows; ++y){
		for(int x = 0; x < a.cols; ++x){
			int summary = 0;
			for(int z = 0; z < a.dims; ++z){
				summary += pow(a.at<cv::Vec3b>(y, x)[z] - b.at<cv::Vec3b>(y, x)[z], 2);
			}
			out.at<uchar>(y, x) = (int) (255.0 * sqrt(summary) / sqrt(3 * pow(255, 2)));
		}
	}
}
