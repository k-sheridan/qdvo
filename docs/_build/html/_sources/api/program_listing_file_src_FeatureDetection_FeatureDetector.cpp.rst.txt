
.. _program_listing_file_src_FeatureDetection_FeatureDetector.cpp:

Program Listing for File FeatureDetector.cpp
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureDetection_FeatureDetector.cpp>` (``src/FeatureDetection/FeatureDetector.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "FeatureDetector.h"
   
   QDVO::FeatureDetector::FeatureDetector()
   {
   
   }
   
   
   std::vector<QDVO::Feature> QDVO::FeatureDetector::detectFeatures(Frame& frame, const int level)
   {
       cv::Mat image = frame.imagePyr.getImage(level).toOpenCVImage();
   
       // First, compute the image gradients.
       cv::Sobel( image, this->dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
       cv::Sobel( image, this->dy, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
   
       this->dxdx.create(image.rows, image.cols, CV_32F);
       this->dxdy.create(image.rows, image.cols, CV_32F);
       this->dydy.create(image.rows, image.cols, CV_32F);
   
       // second precompute the individual structure tensors
       for (int row = 0; row < image.rows; ++row)
       {
           const int16_t *rowPtr_dx = this->dx.ptr<short>(row);
           const int16_t *rowPtr_dy = this->dy.ptr<short>(row);
   
           float *rowPtr_dxdx = this->dxdx.ptr<float>(row);
           float *rowPtr_dxdy = this->dxdy.ptr<float>(row);
           float *rowPtr_dydy = this->dydy.ptr<float>(row);
   
           for (int col = 0; col < image.cols; ++col)
           {
               rowPtr_dxdx[col] = rowPtr_dx[col]*rowPtr_dx[col];
               rowPtr_dydy[col] = rowPtr_dy[col]*rowPtr_dy[col];
               rowPtr_dxdy[col] = rowPtr_dx[col]*rowPtr_dy[col];
           }
        }
   
       // split up the detection into a grid process.
       const int dim = std::floor(sqrt(N_SECTIONS));
       const int maxCandidatesPerSection = ((image.rows * image.cols) / N_SECTIONS) + 1;
       const int nFeaturesPerSection = N_FEATURES_DESIRED / N_SECTIONS;
       std::vector<QDVO::Feature> features;
       features.reserve(N_FEATURES_DESIRED);
   
   
       const double rowsPerSection = double(image.rows) / dim;
       const double colsPerSection = double(image.cols) / dim;
   
       double pad = std::ceil((HARRIS_WIDTH - 1)/2);
   
       std::vector<int> rowBounds, colBounds;
       for(int i = 0; i < dim; ++i)
       {
           rowBounds.push_back(std::max(std::round(i * rowsPerSection), pad));
           colBounds.push_back(std::max(std::round(i * colsPerSection), pad));
       }
   
       rowBounds.push_back(image.rows - pad - 2);
       colBounds.push_back(image.cols - pad - 2);
   
   
       // reset/create the feature candidate vectors.
       std::vector<FeatureCandidate> localCandidates;
       localCandidates.reserve(maxCandidatesPerSection);
   
       const SCALAR_TYPE gradientMagThreshold = (MINIMUM_NORMALIZED_GRADIENT_MAG * frame.maxIntensity());
   
   #if USE_SPATIAL_MASK
       // reset the spatial mask.
       this->spatialMask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
   #endif
   
       // look for the pixels which have a sufficiently high gradient magnitude
       for (std::vector<int>::iterator upperRowBoundIt = rowBounds.begin() + 1; upperRowBoundIt != rowBounds.end(); ++upperRowBoundIt)
       {
           for (std::vector<int>::iterator upperColBoundIt = colBounds.begin() + 1; upperColBoundIt != colBounds.end(); ++upperColBoundIt)
           {
               // empty the local candidate vector
               localCandidates.clear();
   
               const int rl = *(upperRowBoundIt-1);
               const int cl = *(upperColBoundIt-1);
               const int ru = *(upperRowBoundIt);
               const int cu = *(upperColBoundIt);
   
               //std::cout << rl << " " << ru << " " << cl << " " << cu << std::endl;
   
   
               // find all local feature candidates.
               for (int row = rl; row < ru; ++row)
               {
                   for (int col = cl; col < cu; ++col)
                   {
                       FeatureCandidate fc;
   
                       fc.gradientNorm = sqrt(this->dxdx.at<float>(row, col) + this->dydy.at<float>(row, col));
   
                       // If the gradient norm is too low this can never be a feature.
                       if (fc.gradientNorm < gradientMagThreshold)
                       {
                           continue;
                       }
   
                       cv::Rect roi(col, row, HARRIS_WIDTH, HARRIS_WIDTH);
                       fc.dxdx = cv::sum(this->dxdx(roi))[0];
                       fc.dxdy = cv::sum(this->dxdy(roi))[0];
                       fc.dydy = cv::sum(this->dydy(roi))[0];
   
                       fc.trace = fc.dxdx + fc.dydy;
   
                       fc.det = fc.dxdx*fc.dydy + fc.dxdy*fc.dxdy;
   
                       fc.harris = fc.det - HARRIS_K * fc.trace;
   
                       if (fc.harris < 0) {fc.score = -EDGE_WEIGHT * fc.harris;}
                       else {fc.score = fc.harris;}
   
                       fc.x = col;
                       fc.y = row;
   
   
                       localCandidates.push_back(fc); // push onto the list
   
                   }
               }
   
               //std::cout << localCandidates.size() << std::endl;
   
               if (localCandidates.empty()){continue;}
   
               // compute the mean gradient norm
               double sum = 0;
               SCALAR_TYPE maxGradientNorm = 0;
               for (auto e : localCandidates)
               {
                   sum += e.gradientNorm;
                   if (e.gradientNorm > maxGradientNorm){maxGradientNorm = e.gradientNorm;}
               }
   
               SCALAR_TYPE meanGradientNorm = sum / localCandidates.size();
   
               //std::cout << "gradient mean: " << meanGradientNorm << std::endl;
   
               // filter candidates
               for (auto& e : localCandidates)
               {
                   if (e.gradientNorm < gradientMagThreshold)
                   {
                       e.score = 0;
                   }
   
                   if ((e.gradientNorm - meanGradientNorm) / (maxGradientNorm - meanGradientNorm) < INVARIANT_THRESHOLD)
                   {
                       e.score = 0;
                   }
   
               }
   
               //std::cout << "presort" << std::endl;
   #ifndef USE_SPATIAL_MASK
               // find the top n features.
               if (localCandidates.size() >= nFeaturesPerSection)
               {
                   std::partial_sort(localCandidates.begin(), localCandidates.begin() + nFeaturesPerSection, localCandidates.end(), [](const FeatureCandidate &a, const FeatureCandidate &b)
                                 {
                                     return a.score > b.score;
                                 });
               }
   
   
   
               /*for (auto e : localCandidates)
               {
                   std::cout << e.score << std::endl;
               }
               std::cout << "=-=-=-=-=-" << std::endl;
               std::cout << nFeaturesPerSection << std::endl;
               std::cout << "=-=-=-=-=-" << std::endl;*/
   
               for (int i = 1; i <= nFeaturesPerSection && i < localCandidates.size(); ++i)
               {
                   if (bestFeatureIt->score == 0){continue;}
   
                   QDVO::Feature ft;
                   ft.px.x = localCandidates[i].x;
                   ft.px.y = localCandidates[i].y;
   
                   features.push_back(ft);
               }
   #else
               // find the best feature which is not covered by the spatial mask
   
               for (int i = 0; i < nFeaturesPerSection && i < localCandidates.size(); ++i)
               {
                   std::vector<FeatureCandidate>::iterator bestFeatureIt = localCandidates.begin();
                   bool validFeatureFound = false;
                   //std::cout << "asfjdhaslkjdhf" << std::endl;
                   if (this->spatialMask.at<uchar>(cv::Point(bestFeatureIt->x, bestFeatureIt->y)) == 0)
                   {
                       validFeatureFound = true;
                   }
   
                   //std::cout << "kjf" << std::endl;
                   for (std::vector<FeatureCandidate>::iterator it = localCandidates.begin(); it != localCandidates.end(); it++)
                   {
                       if (it->score > bestFeatureIt->score)
                       {
                           if (this->spatialMask.at<uchar>(cv::Point(it->x, it->y)) == 0)
                           {
                               bestFeatureIt = it;
                               validFeatureFound = true;
                           }
                       }
                   }
   
                   //std::cout << "lsakjf" << std::endl;
   
                   if (bestFeatureIt->score == 0){break;}
   
                   if (validFeatureFound)
                   {
                       // apply mask
                       cv::circle(this->spatialMask, cv::Point(bestFeatureIt->x, bestFeatureIt->y), SPATIAL_MASK_RADIUS, cv::Scalar(255), -1);
   
                       // add feature
                       QDVO::Feature ft;
                       ft.px.x = bestFeatureIt->x;
                       ft.px.y = bestFeatureIt->y;
   
                       features.push_back(ft);
   
                   }
                   else
                   {
                       break;
                   }
               }
   
   #endif
   
   
           }
       }
   
   
       return features;
   }
