#include "brain.h"
#include "gotopose.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_GOTOPOSE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterGotoposeNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_GOTOPOSE_BUILDER(GoToPose) // Gotopose
}

NodeStatus GoToPose::tick(){
    double turn_Threshold;
    double stop_Threshold;
    double vLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("v_limit", vLimit);

    // 목표지점
    double targetx, targety, targettheta;
    getInput("target_pos_x", targetx); 
    getInput("target_pos_y", targety); 
    getInput("target_pos_theta", targettheta); 

NodeStatus GoToPose::tick(){
    // 안쓰는 레거시 =================
    double turn_Threshold, targetx, targety, targettheta;
    getInput("turn_threshold", turn_Threshold); 
    getInput("target_pos_x", targetx); 
    getInput("target_pos_y", targety); 
    getInput("target_pos_theta", targettheta); 
    // ============================

    double stop_Threshold;
    double vLimit;
    getInput("stop_threshold", stop_Threshold);
    getInput("v_limit", vLimit);

    // =========================
    // Offtheball "위치 선정" 로직 이식 (targetx, targety, targettheta 계산)
    // =========================
    auto fd = brain->config->fieldDimensions;

    // dist_from_goal은 Offtheball에선 포트로 받지만, GoToPose는 포트 추가 없이 기본값 사용
    double distFromGoal = 3.0;

    double goalX = -(fd.length / 2.0);
    double baseX = goalX + distFromGoal;

    double maxY = fd.width / 2.0 - 0.5;
    double bestX = baseX;
    double bestY = 0.0;
    double maxScore = -1e9;

    // 현재 로봇 위치
    auto rPos = brain->data->robotPoseToField;
    double robotX = rPos.x;
    double robotY = rPos.y;
    double robotTheta = rPos.theta;

    // 상대 선수들
    auto Opponents = brain->data->getRobots();

    // 골대 주변 수비수 인덱스 수집
    std::vector<int> defenderIndices;
    for (size_t idx = 0; idx < Opponents.size(); idx++) {
        if (Opponents[idx].label != "Opponent") continue;
        if (std::abs(Opponents[idx].posToField.x - goalX) < 4.0) {
            defenderIndices.push_back(static_cast<int>(idx));
        }
    }

    // opponent 대칭점 계산
    double symTargetY = 0.0;
    if (!defenderIndices.empty()) {
        double totalOppY = 0.0;
        for (int idx : defenderIndices) {
            totalOppY += Opponents[idx].posToField.y;
        }
        symTargetY = -(totalOppY / defenderIndices.size());
    }

    // 후보 지점 탐색 (Offtheball과 동일)
    for (double x = baseX - 2.0; x <= baseX + 2.0; x += 0.1) {
        for (double y = -maxY; y <= maxY; y += 0.1) {

            double distToDefender = 0.0;
            double normalizer = (defenderIndices.size() > 0 ? defenderIndices.size() : 1.0);

            for (const auto& defenderIndex : defenderIndices) {
                double dist = norm(y - Opponents[defenderIndex].posToField.y,
                                   baseX - Opponents[defenderIndex].posToField.x);
                dist = cap(dist, 3.0, 0.0);
                distToDefender += dist;
            }

            distToDefender /= normalizer;

            double score = 0.0;
            score -= (fabs(x - baseX) * 4.0);
            score -= (fabs(y) * 4.5);
            score -= (fabs(x - robotX) * 2.5);
            score -= (fabs(y - robotY) * 2.5);
            score += (distToDefender * 20.0);

            if (!defenderIndices.empty()) {
                score -= std::abs(y - symTargetY) * 7.5;
            }

            double distXToBall = std::abs(x - brain->data->ball.posToField.x);
            score -= std::abs(distXToBall - 2.5) * 4.5;

            score += (-x) * 0.5;

            Line passPath = {brain->data->ball.posToField.x, brain->data->ball.posToField.y, x, y};
            Line shotPath = {baseX, y, goalX, 0.0};

            for (const auto& opponent : Opponents) {
                if (opponent.label != "Opponent") continue;

                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds();
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0);
                if (confidenceFactor <= 0.0) continue;

                double timeSinceBall = brain->msecsSince(brain->data->ball.timePoint);

                if (timeSinceBall < 3000) {
                    double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                    if (distToPassPath < 1.5) {
                        score -= (1.5 - distToPassPath) * 15.0 * confidenceFactor;
                    }
                }

                double distToShotPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, shotPath);
                if (distToShotPath < 1.5) {
                    score -= (1.5 - distToShotPath) * 3.0 * confidenceFactor;
                }

                Line movementPath = {robotX, robotY, x, y};
                double distRobotTarget = norm(x - robotX, y - robotY);
                if (distRobotTarget > 0.1) {
                    double distToMovementPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, movementPath);
                    if (distToMovementPath < 1.5) {
                        score -= (1.5 - distToMovementPath) * 50.0 * confidenceFactor;
                    }
                }
            }

            double halfGoalW = brain->config->fieldDimensions.goalWidth / 2.0;
            double distToLeftPost = norm(x - goalX, y - halfGoalW);
            double distToRightPost = norm(x - goalX, y + halfGoalW);

            if (distToLeftPost < 0.5) {
                score -= (0.5 - distToLeftPost) * 20.0;
            }
            if (distToRightPost < 0.5) {
                score -= (0.5 - distToRightPost) * 20.0;
            }

            if (score > maxScore) {
                maxScore = score;
                bestX = x;
                bestY = y;
            }
        }
    }

    // Offtheball 결과 목표
    double targetx = bestX;
    double targety = bestY;

    // targettheta도 Offtheball과 동일한 방식으로 계산
    double angleToGoal = atan2(0.0 - robotY, goalX - robotX);
    double angleToBall;
    if (brain->data->ballDetected) {
        angleToBall = brain->data->ball.yawToRobot + robotTheta;
    } else {
        angleToBall = atan2(brain->data->ball.posToField.y - robotY,
                            brain->data->ball.posToField.x - robotX);
    }

    double angleDiff = toPInPI(angleToGoal - angleToBall);
    double targettheta = toPInPI(angleToBall + angleDiff * 0.5);

    // =========================
    // 아래는 GoToPose "기존 이동 제어" 그대로 사용
    // =========================
    double gx = robotX, gy = robotY, gtheta = robotTheta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double errortheta = toPInPI(targettheta - gtheta);

    double dist = norm(errorx, errory);
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

    if(dist > stop_Threshold){
        controltheta = errortheta * Kp;
        controltheta = cap(controltheta, 1.2, -1.2);

        controlx = errorx*cos(gtheta) + errory*sin(gtheta);
        controly = -errorx*sin(gtheta) + errory*cos(gtheta);
        controlx *= linearFactor;
        controly *= linearFactor;
        controlx = cap(controlx, vLimit, -vLimit*0.5);
        controly = cap(controly, vLimit*0.5, -vLimit*0.5);
    }
    else if (fabs(errortheta) > 0.2) {
        controlx = 0;
        controly = 0;
        controltheta = errortheta * Kp;
    }
    else{
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

    brain->client->setVelocity(controlx, controly, controltheta);
    return NodeStatus::SUCCESS;
}