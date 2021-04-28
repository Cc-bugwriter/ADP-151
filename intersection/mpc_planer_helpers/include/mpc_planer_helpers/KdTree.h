#ifndef KdTree_H
#define KdTree_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include "vaafo_msgs/Lane.h"
#include "vaafo_msgs/Waypoint.h"

namespace mpc_planer_helpers{
class KdNode;
class KdTree {

public:
  KdTree();
    virtual ~KdTree();

   KdNode* BuildKDTree(vaafo_msgs::Lane& line);
   int MaxVarianceD(vaafo_msgs::Lane& line);
   void PrintKDTree();
   void FindNearestPoint(const vaafo_msgs::Waypoint& point, vaafo_msgs::Waypoint& npoint,double& distance);
   KdNode* FindLeafNode(KdNode* pnode, const vaafo_msgs::Waypoint& point);
   void FindPCNode(KdNode* pnode, const vaafo_msgs::Waypoint& point, std::vector<KdNode*>& mnpoints, double radius);
   void FindCNode(KdNode* pnode, const vaafo_msgs::Waypoint& point, std::vector<KdNode*>& mnpoints, double radius);
   void FreeNode(KdNode*& pnode);
   double Distance(const vaafo_msgs::Waypoint& point1, const vaafo_msgs::Waypoint& point2);

private:


  int maxv_pos_;
  KdNode* head_;

};

} // end of namespace mpc_planer_helpers

#endif // KdTree_H
