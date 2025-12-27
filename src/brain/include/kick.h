#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

/*
    해당 파일을 킥과 관련된 노드들을 모아놓은 헤더 파일입니다.
*/

void RegisterKickNodes(BT::BehaviorTreeFactory &factory, Brain* brain); // 노드 등록을 위한 함수


// 공과 골대와의 각도를 계산하여 킥 방향을 결정하는 노드
class CalcKickDir : public SyncActionNode {
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){
        return {
            InputPort<double>("cross_threshold", 0.2, "可进门的角度范围小于这个值时, 则传中")
        };
    }

    NodeStatus tick() override;

private:
    NodeStatus tick() override;

private:
    Brain *brain;
};

class CalcPassDir : public SyncActionNode {
public:
    CalcPassDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){
        return {
            InputPort<double>("pass_threshold", 3.0, "팀원과의 최대 거리")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "踢球动作最少执行多少毫秒"),
            InputPort<double>("msecs_stablize", 1000, "稳定多少毫秒"),
            InputPort<double>("speed_limit", 0.8, "速度最大值"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
};