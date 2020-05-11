#include "include/lkorb_tracking.h"

LKORBTracking::LKORBTracking(int width_in,int height_in)
{
    this->width=width_in;
    this->height=height_in;
}


bool LKORBTracking::tracking(CameraFrame& from,
                             CameraFrame& to,
                             vector<Vec2>& lm2d_from,
                             vector<Vec2>& lm2d_to,
                             vector<Vec2>& outlier)
{
    //STEP1: Optical Flow
    int outlier_untracked_cnt=0;
    int outlier_orb_unmatch_cnt=0;

    vector<cv::Point2f> from_cvP2f = from.get2dPtsVec_cvP2f();
    vector<cv::Point2f> tracked_cvP2f;
    vector<cv::Mat>     trackedLMDescriptors;
    vector<float>   err;
    vector<unsigned char> mask_tracked;
    vector<unsigned char> mask_hasorb;
    vector<unsigned char> mask_matched;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

    cv::calcOpticalFlowPyrLK(from.img0, to.img0, from_cvP2f, tracked_cvP2f,
                             mask_tracked, err, cv::Size(31,31), 20, criteria);


    //    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();

    //    vector<cv::KeyPoint>     tracked_lm_cvKP;
    //    cv::Mat descriptorsMat;

    //    cv::KeyPoint::convert(tracked_cvP2f,tracked_lm_cvKP);
    //    extractor->compute(to.img0, tracked_lm_cvKP, descriptorsMat);

    //    for(size_t i=0; i<tracked_cvP2f.size(); i++)
    //    {
    //        unsigned char hasDescriptor = 0;
    //        for(size_t j=0; j<tracked_lm_cvKP.size(); j++)
    //        {
    //            if(tracked_cvP2f.at(i).x==tracked_lm_cvKP.at(j).pt.x &&
    //                    tracked_cvP2f.at(i).y==tracked_lm_cvKP.at(j).pt.y)
    //            {//has ORB descriptor
    //                trackedLMDescriptors.push_back(descriptorsMat.row(j));
    //                hasDescriptor = 1;
    //                break;
    //            }
    //        }
    //        mask_hasorb.push_back(hasDescriptor);
    //        if(hasDescriptor==0)
    //        {
    //            cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, cv::Scalar(0));
    //            trackedLMDescriptors.push_back(zeroDescriptor);
    //        }
    //    }

    //    for(size_t i=0; i<trackedLMDescriptors.size(); i++)
    //    {
    //        if(norm(from.landmarks.at(i).lm_descriptor, trackedLMDescriptors.at(i), cv::NORM_HAMMING) <= 50)
    //        {
    //            //landmarks.at(i).lm_descriptor = trackedLMDescriptors.at(i);
    //            mask_matched.push_back(1);
    //        }else
    //        {
    //            outlier_orb_unmatch_cnt++;
    //            mask_matched.push_back(0);
    //        }
    //    }

    //copy to new frame
    int w=to.width-1;
    int h=to.height-1;
    bool reuslt = false;
    for(int i=from.landmarks.size()-1; i>=0; i--)
    {
        //        if(mask_tracked.at(i)!=1     ||
        //                mask_hasorb.at(i)!=1 ||
        //                mask_matched.at(i)!=1)
        if(mask_tracked.at(i)==1
                &&tracked_cvP2f.at(i).x>0
                &&tracked_cvP2f.at(i).y>0
                &&tracked_cvP2f.at(i).x<w
                &&tracked_cvP2f.at(i).y<h)
        {//inliers
            lm2d_from.push_back(from.landmarks.at(i).lm_2d);
            lm2d_to.push_back(Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y));
            LandMarkInFrame lm=from.landmarks.at(i);
            lm.lm_2d=Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y);
            to.landmarks.push_back(lm);
        }else
        {//outliers
            outlier.push_back(from.landmarks.at(i).lm_2d);
        }
    }

    if(lm2d_to.size()>=20 && outlier.size()>lm2d_to.size())
    {
        reuslt=true;
    }

    return reuslt;
}
