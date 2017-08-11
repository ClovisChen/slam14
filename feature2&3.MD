opencv2
//    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
//    Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
//    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create(matcher_name);

.h:
cv::Ptr<cv::FeatureDetector> detector;  // orb detector
cv::Ptr<cv::DescriptorExtractor> descriptor; // FeatureDetector
.cpp:
detector =  cv::FeatureDetector::create("ORB");
descriptor = cv::DescriptorExtractor::create("ORB");


opencv3
Ptr<FeatureDetector> detector = ORB::create();
Ptr<DescriptorExtractor> descriptor = ORB::create();
Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

.h:
cv::Ptr<cv::FeatureDetector> detector;  // orb detector
cv::Ptr<cv::DescriptorExtractor> descriptor; // FeatureDetector
.cpp:
detector =  cv::ORB::create();
descriptor = cv::ORB::create();

.h:
cv::Ptr<cv::ORB> orb_;
.cpp:
orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
