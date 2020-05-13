#include "include/lkorb_tracking.h"

LKORBTracking::LKORBTracking(int width_in,int height_in)
{
    this->width=width_in;
    this->height=height_in;
}


bool LKORBTracking::tracking(CameraFrame& from,
                             CameraFrame& to,
                             vector<cv::Point2f>& lm2d_from,
                             vector<cv::Point2f>& lm2d_to,
                             vector<cv::Point2f>& outlier)
{
    //STEP1: Optical Flow
    int outlier_untracked_cnt=0;
    int outlier_orb_unmatch_cnt=0;

    vector<cv::Point2f> from_cvP2f = from.get2dPtsVec_cvP2f();
    vector<cv::Point2f> tracked_cvP2f;
    vector<cv::Mat>     trackedLMDescriptors;
    vector<float>   err;
    vector<unsigned char> mask_tracked;
    vector<unsigned char> mask_F_consistant;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

    cv::calcOpticalFlowPyrLK(from.img0, to.img0, from_cvP2f, tracked_cvP2f,
                             mask_tracked, err, cv::Size(31,31), 20, criteria);

    cv::findFundamentalMat(from_cvP2f, tracked_cvP2f, cv::FM_RANSAC, 3, 0.99, mask_F_consistant);
    //copy to new frame
    int w=to.width-1;
    int h=to.height-1;
    bool reuslt = false;
    for(int i=from.landmarks.size()-1; i>=0; i--)
    {
        if(mask_tracked.at(i)==1
                &&mask_F_consistant.at(i)==1
                &&tracked_cvP2f.at(i).x>0
                &&tracked_cvP2f.at(i).y>0
                &&tracked_cvP2f.at(i).x<w
                &&tracked_cvP2f.at(i).y<h)
        {//inliers
            lm2d_from.push_back(from_cvP2f.at(i));
            lm2d_to.push_back(tracked_cvP2f.at(i));
            LandMarkInFrame lm=from.landmarks.at(i);
            lm.lm_2d=Vec2(tracked_cvP2f.at(i).x,tracked_cvP2f.at(i).y);
            to.landmarks.push_back(lm);
        }else
        {//outliers
            outlier.push_back(from_cvP2f.at(i));
        }
    }

    if(lm2d_to.size()>=20 && outlier.size()>lm2d_to.size())
    {
        reuslt=true;
    }

    return reuslt;
}
