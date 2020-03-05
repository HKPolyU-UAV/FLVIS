#include "include/poselmbag.h"
#include <include/common.h>
#include <stdio.h>

PoseLMBag::PoseLMBag()
{
    this->lm_sub_bag.clear();
    wp_init=0;//write pointer for initialization
    pose_cnt_init=0;
    pose_sub_bag_initialized=false;
}

bool PoseLMBag::hasTheLM(int64_t id_in, int &idx)
{
    idx = 0;
    for(int i=0; i<lm_sub_bag.size(); i++)
    {
        if(lm_sub_bag.at(i).id == id_in)
        {
            idx = i;
            return true;
        }
    }
    return false;
}

bool PoseLMBag::addLMObservationSlidingWindow(int64_t id_in, Vec3 p3d_w_in,vector<Proj_LM_ITEM> lms_proj, Vec2 lm_2d)
{
    int idx;
    bool add_lm_to_optimizer=false;
    bool proj = false;
    int proj_num = 0;
    for(size_t i=0; i<lms_proj.size();i++)
    {
      Vec2 dis = lms_proj[i].p2d_proj - lm_2d;
      if(dis.norm() < 3)
      {
        proj = true;
        idx = lms_proj[i].id;
        proj_num++;
      }
    }
   // cout<<"project num on image: "<<proj_num<<endl;

    if(this->hasTheLM(id_in,idx)||proj)
    {//update lm with average 3d inf
        Vec3 p3d_w;
        int cnt = this->lm_sub_bag.at(idx).count;
        cnt++;
        this->lm_sub_bag.at(idx).count = cnt;
    }else{//
        LM_ITEM lm_item;
        lm_item.id = id_in;
        lm_item.count = 1;
        lm_item.p3d_w = p3d_w_in;
        this->lm_sub_bag.push_back(lm_item);
        add_lm_to_optimizer=true;
    }
    return add_lm_to_optimizer;
}
bool PoseLMBag::addLMObservationSlidingWindow(int64_t id_in, Vec3 p3d_w_in)
{
    int idx;
    bool add_lm_to_optimizer=false;


    if(this->hasTheLM(id_in,idx))
    {//update lm with average 3d inf
        Vec3 p3d_w;
        int cnt = this->lm_sub_bag.at(idx).count;
        cnt++;
        this->lm_sub_bag.at(idx).count = cnt;
    }else{//
        LM_ITEM lm_item;
        lm_item.id = id_in;
        lm_item.count = 1;
        lm_item.p3d_w = p3d_w_in;
        this->lm_sub_bag.push_back(lm_item);
        add_lm_to_optimizer=true;
    }
    return add_lm_to_optimizer;
}

bool PoseLMBag::addLMObservation(int64_t id_in, Vec3 p3d_w_in, vector<Proj_LM_ITEM> lms_proj, Vec2 lm_2d)
{
    int idx;
    bool add_lm_to_optimizer=false;

    bool proj = false;
    for(size_t i=0; i<lms_proj.size();i++)
    {
      Vec2 dis = lms_proj[i].p2d_proj - lm_2d;
      if(dis.norm() < 2)
      {
        proj = true;
        idx = lms_proj[i].id;
        break;
      }
    }

    if(this->hasTheLM(id_in,idx) || proj)
    {//update lm with average 3d inf
        Vec3 p3d_w;
        int cnt = this->lm_sub_bag.at(idx).count;
        p3d_w = static_cast<double>(cnt) * this->lm_sub_bag.at(idx).p3d_w + p3d_w_in;
        cnt++;
        p3d_w = (1.0/static_cast<double>(cnt))*p3d_w;
        this->lm_sub_bag.at(idx).count = cnt;
        this->lm_sub_bag.at(idx).p3d_w = p3d_w;
    }else{//
        LM_ITEM lm_item;
        lm_item.id = id_in;
        lm_item.count = 1;
        lm_item.p3d_w = p3d_w_in;
        this->lm_sub_bag.push_back(lm_item);
        add_lm_to_optimizer=true;
    }
    return add_lm_to_optimizer;
}
bool PoseLMBag::addLMObservation(int64_t id_in, Vec3 p3d_w_in)
{
    int idx;
    bool add_lm_to_optimizer=false;


    if(this->hasTheLM(id_in,idx) )
    {//update lm with average 3d inf
        Vec3 p3d_w;
        int cnt = this->lm_sub_bag.at(idx).count;
        p3d_w = static_cast<double>(cnt) * this->lm_sub_bag.at(idx).p3d_w + p3d_w_in;
        cnt++;
        p3d_w = (1.0/static_cast<double>(cnt))*p3d_w;
        this->lm_sub_bag.at(idx).count = cnt;
        this->lm_sub_bag.at(idx).p3d_w = p3d_w;
    }else{//
        LM_ITEM lm_item;
        lm_item.id = id_in;
        lm_item.count = 1;
        lm_item.p3d_w = p3d_w_in;
        this->lm_sub_bag.push_back(lm_item);
        add_lm_to_optimizer=true;
    }
    return add_lm_to_optimizer;
}

bool PoseLMBag::removeLMObservation(int64_t id_in)
{
    bool remove_lm_from_optimizer=false;
    int idx;
    if(this->hasTheLM(id_in,idx))
    {
        lm_sub_bag.at(idx).count--;
        if(lm_sub_bag.at(idx).count==0)
        {
            //cout << "remove from bag" << endl;
            lm_sub_bag.erase(lm_sub_bag.begin() + idx);
            remove_lm_from_optimizer = true;
        }
    }
    return remove_lm_from_optimizer;
}

void PoseLMBag::addPose(int64_t id_in, SE3 pose_in)
{
    if(this->pose_sub_bag_initialized)
    {
        //cover the oldest pose with the newpose
        newest = oldest;
        this->pose_sub_bag[newest].relevent_frame_id = id_in;
        this->pose_sub_bag[newest].pose = pose_in;
        this->oldest++;
        if(this->oldest==POSE_LOOP_BUFFER_SIZE)
        {
            this->oldest = 0;
        }
    }else
    {
        this->pose_sub_bag[wp_init].relevent_frame_id = id_in;
        this->pose_sub_bag[wp_init].pose = pose_in;
        this->pose_sub_bag[wp_init].pose_id = wp_init;
        wp_init++;
        if(this->wp_init==POSE_LOOP_BUFFER_SIZE)
        {
            this->pose_sub_bag_initialized = true;
            this->oldest = 0;
            this->newest = (POSE_LOOP_BUFFER_SIZE-1);
        }
    }
}


void PoseLMBag::getAllLMs(vector<LM_ITEM> &lms_out)
{
    lms_out = this->lm_sub_bag;
}

vector<Proj_LM_ITEM> PoseLMBag::projectLMsOnImage(vector<LM_ITEM> &lms, SE3 T_c_w, double fx,double fy,double cx,double cy,
                                       int width, int height)
{
  vector<Proj_LM_ITEM> res_lm;
  res_lm.clear();

  for(size_t i=0;i<lms.size();i++)
  {
    if(lms.size() - i > 300)
      break;
    LM_ITEM lm = lms[i];
    Vec3 lm_c = T_c_w*lm.p3d_w;
    cout<<lm_c<<endl;
    if(lm_c[2] < 0)
      break;
    else {
      Vector2d res, res_;
      res(0) = lm_c(0)/lm_c(2);
      res(1) = lm_c(1)/lm_c(2);
      res_(0) = res(0)*fx + cx;
      res_(1) = res(1)*fy + cy;
      cout<<"px: "<<res_(0)<<" py: "<<res_(1)<<endl;
      if(res_(0) < 10 || res_(0)> width - 10 || res_(1) < 10 || res_(1) > height - 10)
        break;
      else {
        Proj_LM_ITEM lm_;
        lm_.id = lm.id;
        lm_.p3d_w = lm.p3d_w;
        lm_.count = lm.count;
        lm_.p2d_proj = res_;
        res_lm.push_back(lm_);
      }

    }
  }
  cout<<"in image boundry "<<res_lm.size()<<endl;
  return res_lm;
}

void PoseLMBag::getMultiViewLMs(vector<LM_ITEM> &lms_out, int view_cnt)
{
    lms_out.clear();
    for(auto lm_in_bag:lm_sub_bag)
    {
        if(lm_in_bag.count>=view_cnt)
        {
            lms_out.push_back(lm_in_bag);
        }
    }
}

void PoseLMBag::getAllPoses(vector<POSE_ITEM> &poses_out)
{
    poses_out.clear();
    for (int i=0; i<POSE_LOOP_BUFFER_SIZE; i++) {
        poses_out.push_back(this->pose_sub_bag[i]);
    }
}

int PoseLMBag::getNewestPoseInOptimizerIdx(void)
{
    return this->newest;
}

int PoseLMBag::getOldestPoseInOptimizerIdx(void)
{
    return this->oldest;
}


int64_t PoseLMBag::getPoseIdByReleventFrameId(int64_t frame_id)
{
    int64_t ret_val=-1;
    for(int i=0; i<POSE_LOOP_BUFFER_SIZE; i++)
    {
        if(pose_sub_bag[i].relevent_frame_id == frame_id)
        {
            ret_val = i;
            break;
        }
    }
    return ret_val;
}

void PoseLMBag::debug_output(void)
{
    cout << "debug output" << endl;
    cout << "Poses:" << endl;
    for(int i=0; i<POSE_LOOP_BUFFER_SIZE; i++)
    {
        cout << "pose id " << pose_sub_bag[i].pose_id
             << " relevent frame id: " << pose_sub_bag[i].relevent_frame_id
             << " Twc: " << pose_sub_bag[i].pose.inverse().so3().log().transpose()
             << " | " << pose_sub_bag[i].pose.inverse().translation().transpose() << endl;;
    }
    cout << "LMs:" << endl;
    for(std::vector<LM_ITEM>::iterator it = this->lm_sub_bag.begin(); it != this->lm_sub_bag.end(); ++it)
    {
        cout << "lm id" << it->id
             << " count " << it->count
             << " p3d: " << it->p3d_w.transpose() << endl;
    }

}
