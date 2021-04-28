#include <mpc_planer_helpers/KdTree.h>
#include <queue>

namespace mpc_planer_helpers{

class KdNode{
public:
  KdNode(){
    parent_ = nullptr;
    left_ = nullptr;
    right_ = nullptr;

    is_leaf_ = false;
  }

  ~KdNode(){

  }
  vaafo_msgs::Waypoint point_;
  KdNode* parent_;
  KdNode* left_;
  KdNode* right_;
  int split;
  bool is_leaf_;

};




KdTree::KdTree()
{   
  head_ = nullptr;
  maxv_pos_ = -1;
}


KdTree::~KdTree()
{
  FreeNode(head_);
}


int KdTree::MaxVarianceD(vaafo_msgs::Lane& line){

  if(line.waypoints.empty()) return -1;

  double sum[2]={0}, sq_sum[2]={0};
  double mean_v[2]={0};
  std::vector<double> variance(2,0);

  for (size_t j = 0;j<line.waypoints.size();j++) {
        sum[0]+=line.waypoints.at(j).pose.pose.position.x;
        sum[1]+=line.waypoints.at(j).pose.pose.position.y;
        sq_sum[0]+=std::pow(line.waypoints.at(j).pose.pose.position.x,2);
        sq_sum[1]+=std::pow(line.waypoints.at(j).pose.pose.position.y,2);
  }

  for(int i=0;i<2;i++){
  mean_v[i] = sum[i]/line.waypoints.size();
  variance[i]=sq_sum[i]/line.waypoints.size() - std::pow(mean_v[i],2);
  }

  auto maxv_pos = std::max_element(variance.begin(),variance.end());

  return  maxv_pos - variance.begin();

}

KdNode* KdTree::BuildKDTree(vaafo_msgs::Lane& line){

  if(line.waypoints.empty()) return nullptr;

  //acquire the variance and dimension
  maxv_pos_ = MaxVarianceD(line);
  if (maxv_pos_<0) return nullptr;

  //sort and get medium value
  vaafo_msgs::Lane& mline = const_cast<vaafo_msgs::Lane&>(line);

  std::sort(mline.waypoints.begin(),mline.waypoints.end(),[this](vaafo_msgs::Waypoint& left, vaafo_msgs::Waypoint& right){
    if(maxv_pos_== 0) return left.pose.pose.position.x < right.pose.pose.position.x;
    else {
      return left.pose.pose.position.y < right.pose.pose.position.y;
    }

  });

  unsigned long mid_idx = mline.waypoints.size()/2;

  KdNode* pnode = new KdNode();
  pnode->point_ = mline.waypoints.at(mid_idx);

  //devide left and right space
  vaafo_msgs::Lane left_line,right_line;
  left_line.waypoints.insert(left_line.waypoints.end(),mline.waypoints.begin(),mline.waypoints.begin()+mid_idx);
  right_line.waypoints.insert(right_line.waypoints.end(),mline.waypoints.begin()+mid_idx+1,mline.waypoints.end());

  pnode->split = maxv_pos_;
  pnode->left_ = BuildKDTree(left_line);
  pnode->right_ = BuildKDTree(right_line);

  //define parent node
  if(pnode->left_ != nullptr) pnode->left_->parent_ = pnode;
  if(pnode->right_ !=nullptr) pnode->right_->parent_ = pnode;

  //judge whether the node is leaf node
  if (pnode->left_ == nullptr && pnode->right_ == nullptr) pnode->is_leaf_ = true;

  head_ = pnode;
  return pnode;

}

void KdTree::PrintKDTree(){
  std::queue<std::pair<KdNode*, int> > nodes;
  nodes.push(std::pair<KdNode*, int>(head_, 0));

  int last_layer = 0;
  while (!nodes.empty()){
    auto data = nodes.front();
    if (data.first != nullptr){
      if (data.second > last_layer)
      {
        last_layer = data.second;

        //change line
        std::cout << std::endl;
      }

      if (data.second < last_layer)
      {
        //left and right of a parent node is finished
        std::cout << data.first->point_.pose.pose.position.x<< "," << data.first->point_.pose.pose.position.y << "\t";
      }
      else
      {
        std::cout << data.first->point_.pose.pose.position.x<< "," << data.first->point_.pose.pose.position.y << " ";
      }

      nodes.push(std::pair<KdNode*, int>(data.first->left_, data.second + 1));
      nodes.push(std::pair<KdNode*, int>(data.first->right_, data.second));

    }
     nodes.pop();
  }

}

void KdTree::FindNearestPoint(const vaafo_msgs::Waypoint& point, vaafo_msgs::Waypoint& npoint, double& distance){

  //nearest point
  std::vector<KdNode*> mnpoints;
  std::vector<KdNode*> search_path;

 vaafo_msgs::Waypoint nearest;
  double dist;

  KdNode* pSearch = head_;

  //find leaf point, considered as nearear point
  while (pSearch != nullptr) {

     //add pSearch to search_path
    search_path.push_back(pSearch);

    if(pSearch->split == 0){
      if(point.pose.pose.position.x <= pSearch->point_.pose.pose.position.x){
        pSearch = pSearch->left_;
      }
      else {
        pSearch = pSearch->right_;
      }

    }else {
      if(point.pose.pose.position.y <= pSearch->point_.pose.pose.position.y){
        pSearch = pSearch->left_;
      }else {
        pSearch = pSearch->right_;
      }
    }
   }

  //assign last one in search_path into nearest
  nearest = search_path.back()->point_;
  search_path.pop_back();

  dist = Distance(nearest,point);

  //search back
  KdNode* pBack;

  while (search_path.size() !=0 ) {
    pBack = search_path.back();
    search_path.pop_back();

    if(pBack->left_ == nullptr && pBack->right_ == nullptr){
      if(Distance(nearest,point) > Distance(pBack->point_, point)){
         nearest = pBack->point_;
         dist = Distance(pBack->point_, point);
      }

    }else {

      if(pBack->split == 0){
        //distance smaller than dist, both sides should be searched
        if(abs(pBack->point_.pose.pose.position.x - point.pose.pose.position.x) < dist){
          if(Distance(nearest,point) > Distance(pBack->point_,point)){
            nearest = pBack->point_;
            dist = Distance(pBack->point_,point);
          }
            if(pBack->right_ != nullptr){search_path.push_back(pBack->right_);}
            if(pBack->left_ != nullptr){search_path.push_back(pBack->left_);}
        }else {
          if(point.pose.pose.position.x <= pBack->point_.pose.pose.position.x) //search other side
            pSearch = pBack->right_;
          else
            pSearch = pBack->left_;
          if(pSearch != nullptr){search_path.push_back(pSearch);}
          }
      //for other dimension
      }else {
        if(abs(pBack->point_.pose.pose.position.y - point.pose.pose.position.y) < dist){
          if(Distance(nearest,point) > Distance(pBack->point_,point)){
            nearest = pBack->point_;
            dist = Distance(pBack->point_,point);
          }
          if(pBack->right_ != nullptr){search_path.push_back(pBack->right_);}
          if(pBack->left_ != nullptr){search_path.push_back(pBack->left_);}
        }else {
          if(point.pose.pose.position.y <= pBack->point_.pose.pose.position.y)
            pSearch = pBack->right_;
          else
            pSearch = pBack->left_;
          if(pSearch != nullptr){search_path.push_back(pSearch);}
        }
      }



    }

  }

  npoint = nearest;
  distance = dist;


}


double KdTree::Distance(const vaafo_msgs::Waypoint &point1, const vaafo_msgs::Waypoint &point2){
  return hypot(point1.pose.pose.position.x - point2.pose.pose.position.x, point1.pose.pose.position.y - point2.pose.pose.position.y);
}



void KdTree::FreeNode(KdNode*& pnode)
{
  if(pnode != nullptr)
  {
    FreeNode(pnode->left_);
    FreeNode(pnode->right_);

    delete pnode;
    pnode = nullptr;
  }
}

} // end of namespace mpc_planer_helpers
