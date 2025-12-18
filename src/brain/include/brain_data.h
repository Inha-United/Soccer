#pragma once

#include <string>
#include <mutex>
#include <tuple>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"
#include <Eigen/Dense> 

#include "types.h"
#include "RoboCupGameControlData.h"

using namespace std;

struct BrainData {
    BrainData();

    // -- 게임 컨트롤러 관련 데이터 -- 
    int score = 0;
    int oppoScore = 0;

    int penalty[HL_MAX_NUM_PLAYERS]; // 우리 팀 선수의 페널티 상태
    int oppoPenalty[HL_MAX_NUM_PLAYERS]; // 상대 팀 선수의 페널티 상태

    bool isKickingOff = false; // 킥오프(선공) 팀인지 여부
    rclcpp::Time kickoffStartTime; // 킥오프 시작 시간
    bool isFreekickKickingOff = false; // 프리킥 킥오프 팀인지 여부
    rclcpp::Time freekickKickoffStartTime; // 프리킥 킥오프 시작 시간
    int liveCount = 0; // 경기 가능한 선수 수
    int oppoLiveCount = 0; // 상대 경기 가능한 선수 수
    string realGameSubState; // 현재 게임의 하위 상태

    // 게임 상태 관련
    bool isDirectShoot = false; // 패널티킥에서 직접 슛팅으로 해도 되는지


    // -- locator 관련 데이터 --
    Pose2D robotPoseToOdom;  
    Pose2D odomToField;      
    Pose2D robotPoseToField; 
     
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity(); 

    // 아래 함수들은 bain.cpp/calibrateOdom에서 사용됨
    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }

    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }

    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }

    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }

    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]; 
    int tmCmdId = 0; 
    rclcpp::Time tmLastCmdChangeTime; 
    int tmMyCmd = 0; 
    int tmMyCmdId = 0; 
    int tmReceivedCmd = 0; 
    bool tmImLead = true; 
    bool tmImAlive = true; 
    double tmMyCost = 0.;

    rclcpp::Time timeLastDet; 
    bool camConnected = false; 
    rclcpp::Time timeLastLineDet; 
    rclcpp::Time lastSuccessfulLocalizeTime;
    rclcpp::Time timeLastGamecontrolMsg; 
    rclcpp::Time timeLastLogSave; 
    VisionBox visionBox;  
    rclcpp::Time lastTick; 

    /**
     * @brief 타입에 따라 markings를 조회한다.
     * 
     * @param type set<string>
     *        빈 set인 경우 모든 타입을 의미하며,
     *        비어 있지 않은 경우 "LCross" | "TCross" | "XCross" | "PenaltyPoint" 중
     *        지정된 타입의 markings만 조회한다.
     * 
     * @return vector<GameObject> 지정된 타입에 해당하는 markings
     */

    vector<GameObject> getMarkingsByType(set<string> types={});


    vector<FieldMarker> getMarkersForLocator();


    Pose2D robot2field(const Pose2D &poseToRobot);


    Pose2D field2robot(const Pose2D &poseToField);

private:
    vector<GameObject> _robots = {}; 
    mutable std::mutex _robotsMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

};
