/*
 * KLTTracker.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */

#include <KLTTracker.h>

KLTTracker::KLTTracker()
{
	// TODO Auto-generated constructor stub

}

KLTTracker::~KLTTracker()
{
	// TODO Auto-generated destructor stub
}

/*
 * the main klt tracker function
 * iteratively determines the new sub-pixel accurate feature positions in the new frame and estimates its covariance matrix
 *
 * the initial version of this code is not vectorized by me and ideally gets auto-vectorized by the compiler
 *
 * all inputs and outputs are in homogenous metric coordinates
 *
 * NOTE:
 * for best results the cf should contain the maximum likelihood predicted pose
 */
void KLTTracker::findNewFeaturePositions(const Frame& lf, Frame& cf)
{
	// for now I use the opencv built in klt tracker with custom uncertainty estimation
	this->findNewFeaturePositionsOpenCV(lf, cf);
}

/*
 * uses the built in opencv klt tracker and estimates the uncertainty of the results
 */
void KLTTracker::findNewFeaturePositionsOpenCV(const Frame& lf, Frame& cf)
{

	ROS_ASSERT(cf.features.size() == lf.features.size());

	if(!lf.features.size()){return;}

	std::vector<cv::Point2f> prev_fts, new_fts;
	std::vector<uchar> status; // status vector for each point
	cv::Mat error; // error vector for each point

	//load the vectors
	for(auto e : lf.features){
		prev_fts.push_back(e.px);

	}
	for(auto e : cf.features){
		// essentially this projects the feature into this frame's predicted pose
		cv::Point2f px_prediction = Feature::metric2Pixel(cf.K, Feature::point2bearingAndzinv(e.projectFeature(cf.pose)).block<2, 1>(0, 0));

		new_fts.push_back(px_prediction);
	}

	cv::calcOpticalFlowPyrLK(lf.img, cf.img, prev_fts, new_fts,
			status, error, cv::Size(WINDOW_SIZE, WINDOW_SIZE), MAX_PYRAMID_LEVEL,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					30, 0.01), 0, KLT_MIN_EIGEN);

/*	cv::calcOpticalFlowPyrLK(lf.img, cf.img, prev_fts, new_fts,
				status, error, cv::Size(WINDOW_SIZE, WINDOW_SIZE), MAX_PYRAMID_LEVEL,
				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW, KLT_MIN_EIGEN);*/


	int lost_features = 0;

	// set the pixel position measurement
	int i = 0;
	for(std::list<Feature>::iterator it = cf.features.begin(); it != cf.features.end() && i < status.size();i++){
		// check if the feature was flowed properly
		if(status.at(i)){
			//TODO check for occlusion and add the patch
			cv::Mat patch = Feature::extractPatch(new_fts.at(i), cf.img);

			

			it->px = new_fts.at(i);
			it->R_inv = this->estimateUncertainty(cf, it->px).inverse();

			it++;
		}
		else
		{
			// delete this feature because it has not been successfully flowed
			it = cf.features.erase(it);
			//it->to_be_deleted = true;
			lost_features++;
		}
	}

	ROS_DEBUG_STREAM("lost " << lost_features << " features during klt");
}

/*
 * estimates the uncertainty of the feature position in pixel units
 */
Eigen::Matrix2f KLTTracker::estimateUncertainty(const Frame& cf, cv::Point2f mu)
{
	ROS_ASSERT(cf.img.rows || mu.x);
	Eigen::Matrix2f A;
	A << 0.00001, 0, 0, 0.00001;
	return A;
}

/*
 * samples a small region around the feature with its reference to estimate the covariance matrix
 */
Eigen::Matrix2f KLTTracker::estimateUncertaintySampleBased(const Frame& lf, cv::Point2f mu_ref, const Frame& cf, cv::Point2f mu){
	Eigen::Matrix2f A;

	cv::Mat ref;

	//TODO check if feature is too close to the boundaries

	float window_size = 5;

	cv::getRectSubPix(lf.img, cv::Size(window_size, window_size), mu_ref, ref, CV_32F); // subpixel reference patch

	float window_area = window_size*window_size;
	float k = 0.01;

	float sum_rd=0;
	float sum_rd_xx=0;
	float sum_rd_yy=0;
	float sum_rd_xy=0; // = yx

	//compute the gaussian with a 5x5 sample
	for(float du = -10; du <= 10; du+=5)
	{
		for(float dv = -10; dv <= 10; dv+=5)
		{
			cv::Point2f sample_mu = mu + cv::Point2f(du, dv);

			//compute the subpixel image patch for this sample
			cv::Mat sample;

			cv::getRectSubPix(cf.img, cv::Size(window_size, window_size), sample_mu, sample, CV_32F); // subpixel test patch

			//compute the ssd for this patch
			float ssd = 0;
			for(int i = 0; i < window_size; i++){
				for(int j = 0; j < window_size; j++){
					ssd += pow(ref.at<float>(i, j) - sample.at<float>(i, j), 2);
				}
			}

			ssd /= window_area; // normalize

			//ROS_DEBUG_STREAM("ssd: " << ssd);

			float rd = exp(-k*ssd);

			sum_rd += rd;
			sum_rd_xx += rd*du*du;
			sum_rd_yy += rd*dv*dv;
			sum_rd_xy += rd*du*dv;
		}
	}

	//finally construct the covariance matrix

	//ROS_DEBUG_STREAM("sum_rd: " << sum_rd);

	A(0, 0) = sum_rd_xx/sum_rd;
	A(1, 1) = sum_rd_yy/sum_rd;
	A(0, 1) = sum_rd_xy/sum_rd;
	A(1, 0) = A(0, 1);

	//ROS_DEBUG_STREAM("cov: " << A);

	return A;
}
