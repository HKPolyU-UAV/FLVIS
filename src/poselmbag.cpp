#include "include/poselmbag.h"

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
void PoseLMBag::addLM(int64_t id_in, Vec3 p3d_w_in)
{
    int idx;
    if(this->hasTheLM(id_in,idx))
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
    }
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
             << " relevent frame id: " << pose_sub_bag[i].relevent_frame_id << endl
             << pose_sub_bag[i].pose << endl;;
    }
    cout << "LMs:" << endl;
    for(std::vector<LM_ITEM>::iterator it = this->lm_sub_bag.begin(); it != this->lm_sub_bag.end(); ++it)
    {
        cout << "lm id" << it->id
             << " count " << it->count
             << " p3d: " << it->p3d_w.transpose() << endl;
    }

}
