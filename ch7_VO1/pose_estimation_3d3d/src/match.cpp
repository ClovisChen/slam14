//
// Created by chen-tian on 7/24/17.
//

#include "match.h"
using  namespace std;
void match::find_feature_matches( const Mat& img_1, const Mat& img_2,
                           std::vector<KeyPoint>& keypoints_1,
                           std::vector<KeyPoint>& keypoints_2,
                           std::vector< DMatch >& matches )
{
    //-- initialization
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- the first: Detect oriented FAST corners location
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- the second: compute dscriptor
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- the third: use "Hamming" to match BRIEF descriptor between two images
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- the forth: filter the match point
    double min_dist=10000, max_dist=0;

    //find the max_distance and min_distance between all matches
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //when the distance of descriptor is twice than min_distance, the match is wrong
    // 30 is the experience lower_limit
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}