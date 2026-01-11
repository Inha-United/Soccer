#include "pass_receive.h"
#include "utils/math.h"

using namespace std;
using namespace BT;

NodeStatus PassReceive::onStart()
{
    return NodeStatus::RUNNING;
}

NodeStatus PassReceive::onRunning()
{
    // 1. Check pass signal from teammates
    bool passSignal = false;
    double targetX = 0.0;
    double targetY = 0.0;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++)
    {
        if (brain->data->tmStatus[i].passSignal)
        {
            passSignal = true;
            targetX = brain->data->tmStatus[i].passTargetX;
            targetY = brain->data->tmStatus[i].passTargetY;
            break; // Found a passer
        }
    }

    if (!passSignal)
    {
        // No pass signal, maybe just stop or return failure?
        // But if this node is called, it means Decision decided to receive.
        // It's possible the signal was lost momentarily.
        // Stay idle or minimal movement?
        brain->set_velocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    // 2. Move to target
    // Convert target (Field) to Robot coordinates
    double target_rx, target_ry, __;
    transCoord(targetX, targetY, 0, 
               brain->data->robotPoseToField.x, brain->data->robotPoseToField.y, brain->data->robotPoseToField.theta, 
               target_rx, target_ry, __);

    double distToTarget = sqrt(target_rx * target_rx + target_ry * target_ry);
    
    // Avoidance (Basic)
    double targetDir = atan2(target_ry, target_rx);
    double distToObstacle = brain->distToObstacle(targetDir);
    double oaSafeDist = 0.5;

    double vx, vy, vtheta;

    if (distToObstacle < oaSafeDist) {
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        double speed = 0.3;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
    } else {
        double p_gain = 1.0;
        vx = target_rx * p_gain;
        vy = target_ry * p_gain;
    }

    // Facing the ball (or passer) while waiting
    // If ball detected, face ball. Else face passer? 
    // Usually face ball direction to be ready to receive.
    if (brain->data->ballDetected) {
        vtheta = brain->data->ball.yawToRobot * 1.5;
    } else {
        vtheta = targetDir * 1.0; // Look at target? No, look at ball direction ideally.
    }

    // Speed limits
    double maxSpeed = 0.8;
    double speed = sqrt(vx*vx + vy*vy);
    if (speed > maxSpeed) {
        vx = vx / speed * maxSpeed;
        vy = vy / speed * maxSpeed;
    }

    // Arrived?
    if (distToTarget < 0.2) {
        vx = 0; vy = 0;
        // 4. Check if ball received
        if (brain->data->ballDetected && brain->data->ball.range < 0.4) {
             // 5. Return success (pass received)
             brain->set_velocity(0, 0, 0);
             return NodeStatus::SUCCESS;
        }
    }

    brain->set_velocity(vx, vy, vtheta);

    return NodeStatus::RUNNING;
}

void PassReceive::onHalted()
{
}

#define REGISTER_PASSRECEIVE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterPassReceiveNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_PASSRECEIVE_BUILDER(PassReceive)
}
