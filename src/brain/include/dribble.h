#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDribbleNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "공과의 거리가 이 값보다 가까우면 더 이상 공을 향해 이동하지 않음"),
            InputPort<double>("stop_angle", 0.1, "공의 각도가 이 값 이내이면 더 이상 공을 향해 회전하지 않음"),
            InputPort<double>("vy_limit", 0.2, "보행 안정성을 위해 Y 방향 속도를 제한하며 효과를 내려면 로봇 최대 속도 0.4보다 작아야 함"),
            InputPort<double>("vx_limit", 0.6, "보행 안정성을 위해 X 방향 속도를 제한하며 효과를 내려면 로봇 최대 속도 1.2보다 작아야 함"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

