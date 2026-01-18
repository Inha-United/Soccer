#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <string>

class Brain; 
using namespace BT;
using namespace std;


void RegisterOfftheballNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class OffTheBall : public SyncActionNode
{
public:
    OffTheBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("v_limit", 0.5, "최대 속도"),
            InputPort<double>("dist_from_goal", 3.0, "goal 앞에서 대기할 거리"),
        };
    }

    NodeStatus tick() override;


private:
    Brain *brain;
    //std::chrono::steady_clock::time_point scanStartTime = std::chrono::steady_clock::time_point::min();
    //double smoothHeadYaw = 0.0;
};