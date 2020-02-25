#include <include/feature_dem.h>
#include <opencv2/features2d/features2d.hpp>

// Driver function to sort the vector elements by
// second element of pair in descending order
static bool sortbysecdesc(const pair<cv::Point2f,float> &a,
                          const pair<cv::Point2f,float> &b)
{
    return a.second>b.second;
}

FeatureDEM::FeatureDEM(const int image_width,
                       const int image_height,
                       int boundaryBoxSize)
{
    width=image_width;
    height=image_height;
    regionWidth  = floor(width/4.0);
    regionHeight = floor(height/4.0);
    boundary_dis = floor(boundaryBoxSize/2.0);
    int gridx[5],gridy[5];
    for(int i=0; i<5; i++)
    {
        gridx[i]=i*regionWidth;
        gridy[i]=i*regionHeight;
    }
    for(int i=0; i<16; i++)
    {
        detectorMask[i] = cv::Mat(image_height, image_width, CV_8S,cv::Scalar(0));
        int x_begin,x_end,y_begin,y_end;
        x_begin = gridx[i%4];
        x_end   = gridx[(i%4)+1];
        y_begin = gridy[i/4];
        y_end   = gridy[(i/4)+1];
        for(int xx=x_begin; xx<x_end; xx++)
        {
            for(int yy=y_begin; yy<y_end; yy++)
            {
                detectorMask[i].at<schar>(yy,xx)=1;
            }
        }
    }
}

FeatureDEM::~FeatureDEM()
{;}

void FeatureDEM::calHarrisR(const cv::Mat& img,
                            cv::Point2f& Pt,
                            float &R)
{
    uchar patch[9];
    int xx = Pt.x;
    int yy = Pt.y;
    patch[0]=img.at<uchar>(cv::Point(xx-1,yy-1));
    patch[1]=img.at<uchar>(cv::Point(xx,yy-1));
    patch[2]=img.at<uchar>(cv::Point(xx+1,yy-1));
    patch[3]=img.at<uchar>(cv::Point(xx-1,yy));
    patch[4]=img.at<uchar>(cv::Point(xx,yy));
    patch[5]=img.at<uchar>(cv::Point(xx+1,yy+1));
    patch[6]=img.at<uchar>(cv::Point(xx-1,yy+1));
    patch[7]=img.at<uchar>(cv::Point(xx,yy+1));
    patch[8]=img.at<uchar>(cv::Point(xx+1,yy+1));
    float IX,IY;
    float X2,Y2;
    float XY;
    IX = (patch[0]+patch[3]+patch[6]-(patch[2]+patch[5]+patch[8]))/3;
    IY = (patch[0]+patch[1]+patch[2]-(patch[6]+patch[7]+patch[8]))/3;
    X2 = IX*IX;
    Y2 = IY*IX;
    XY = IX*IX;
    //M = | X2  XY |
    //    | XY  Y2 |
    //R = det(M)-k(trace^2(M))
    //  = X2*Y2-XY*XY  - 0.05*(X2+Y2)*(X2+Y2)
    R = (X2*Y2)-(XY*XY) - 0.05*(X2+Y2)*(X2+Y2);
}

void FeatureDEM::filterAndFillIntoRegion(const cv::Mat& img,
                                         const vector<cv::Point2f>& pts)
{
    //Devided all features into 16 regions
    for(size_t i=0; i<pts.size(); i++)
    {
        cv::Point2f pt = pts.at(i);
        if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
        {
            float Harris_R;
            calHarrisR(img,pt,Harris_R);
            //            if(Harris_R>10.0)
            if(1)
            {
                int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
                regionKeyPts[regionNum].push_back(make_pair(pt,Harris_R));
            }
        }
    }
}

void FeatureDEM::fillIntoRegion(const cv::Mat& img,
                                const vector<cv::Point2f>& pts)
{
    for(size_t i=0; i<pts.size(); i++)
    {
        cv::Point2f pt = pts.at(i);
        int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
        regionKeyPts[regionNum].push_back(make_pair(pt,99999.0));
    }
}


void FeatureDEM::redetect(const cv::Mat& img,
                          const vector<Vec2>& existedPts,
                          vector<Vec2>& newPts,
                          vector<cv::Mat>& newDescriptors,
                          int &newPtscount)
{
    //Clear
    newPts.clear();
    newDescriptors.clear();
    newPtscount=0;

    vector<cv::Point2f> newPts_cvP2f;
    newPts_cvP2f.clear();

    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }
    vector<cv::Point2f> existedPts_cvP2f=vVec2_2_vcvP2f(existedPts);
    fillIntoRegion(img,existedPts_cvP2f);
    cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create();

    vector<cv::KeyPoint> features;
    vector<cv::Point2f>  kps;
    detector->detect(img, features);
    cv::KeyPoint::convert(features,kps);
    vector<pair<cv::Point2f,float>> regionKeyPts_prepare[16];
    //devide into different region
    for(size_t i=0; i<kps.size(); i++)
    {
        cv::Point2f pt = kps.at(i);
        if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
        {
            float Harris_R;
            calHarrisR(img,pt,Harris_R);
            if(1)
            {
                int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
                regionKeyPts_prepare[regionNum].push_back(make_pair(pt,Harris_R));
            }
        }
    }
    for(size_t i=0; i<16; i++)
    {
        sort(regionKeyPts_prepare[i].begin(), regionKeyPts_prepare[i].end(), sortbysecdesc);
        for(size_t j=0; j<regionKeyPts_prepare[i].size(); j++)
        {
            int noFeatureNearby = 1;
            cv::Point pt=regionKeyPts_prepare[i].at(j).first;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(pt.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(pt.y-regionKeyPts[i].at(k).first.y);
                if(dis_x <= boundary_dis || dis_y <= boundary_dis)
                {
                    noFeatureNearby=0;
                }
            }
            if(noFeatureNearby)
            {
                regionKeyPts[i].push_back(make_pair(pt,999999.0));
                newPts_cvP2f.push_back(pt);
                if(regionKeyPts[i].size() >= MAX_REGION_FREATURES_NUM) break;
            }
        }
    }

    if(newPts_cvP2f.size()>0)
    {
        cv::Mat tmpDescriptors;
        vector<cv::KeyPoint> tmpKPs;
        cv::KeyPoint::convert(newPts_cvP2f,tmpKPs);
        cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
        extractor->compute(img, tmpKPs, tmpDescriptors);
        for(size_t i=0; i<tmpKPs.size(); i++)
        {
            Vec2 pt(tmpKPs.at(i).pt.x,tmpKPs.at(i).pt.y);
            newPts.push_back(pt);
        }
        newPtscount=newPts.size();
        descriptors_to_vMat(tmpDescriptors,newDescriptors);
    }
}



void FeatureDEM::detect(const cv::Mat& img, vector<Vec2>& pts, vector<cv::Mat>& descriptors)
{
    cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create();
    //Clear
    pts.clear();
    descriptors.clear();
    for(int i=0; i<16; i++)
    {
        regionKeyPts[i].clear();
    }

    //Detect FAST
    vector<cv::KeyPoint> tmpKPs;
    detector->detect(img, tmpKPs);

    //Fill into region
    vector<cv::Point2f>  tmpPts;
    cv::KeyPoint::convert(tmpKPs,tmpPts);
    filterAndFillIntoRegion(img,tmpPts);

    //For every region, select features by Harris index and boundary size
    for(int i=0; i<16; i++)
    {
        sort(regionKeyPts[i].begin(), regionKeyPts[i].end(), sortbysecdesc);
        vector<pair<cv::Point2f,float>> tmp = regionKeyPts[i];
        regionKeyPts[i].clear();
        int count = 0;
        for(size_t j=0; j<tmp.size(); j++)
        {
            int outSideConflictBoundary = 1;
            for(size_t k=0; k<regionKeyPts[i].size(); k++)
            {
                float dis_x = fabs(tmp.at(j).first.x-regionKeyPts[i].at(k).first.x);
                float dis_y = fabs(tmp.at(j).first.y-regionKeyPts[i].at(k).first.y);
                if(dis_x<=boundary_dis || dis_y<=boundary_dis)
                {
                    outSideConflictBoundary=0;
                }
            }
            if(outSideConflictBoundary)
            {
                regionKeyPts[i].push_back(tmp.at(j));
                count++;
                if(count>=MAX_REGION_FREATURES_NUM) break;
            }
        }
    }

    tmpPts.clear();
    for(int i=0; i<16; i++)
    {
        //cout << regionKeyPts[i].size() << "in Region " << i << endl;
        for(size_t j=0; j<regionKeyPts[i].size(); j++)
        {
            tmpPts.push_back(regionKeyPts[i].at(j).first);
        }
    }

    cv::Mat tmpDescriptors;
    cv::KeyPoint::convert(tmpPts,tmpKPs);
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create(2000);
    extractor->compute(img, tmpKPs, tmpDescriptors);

    for(size_t i=0; i<tmpKPs.size(); i++)
    {
        pts.push_back(Vec2(tmpKPs.at(i).pt.x,tmpKPs.at(i).pt.y));
    }
    descriptors_to_vMat(tmpDescriptors,descriptors);
}
