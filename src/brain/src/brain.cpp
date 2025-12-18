#include <iostream>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "brain.h"
#include "detection_utils.h"

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

Brain::Brain() : rclcpp::Node("brain_node"){
    // tf 브로드캐스터 초기화
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 매개변수는 반드시 여기에서 먼저 선언해야 한다는 점에 유의해야 한다.
    // 그렇지 않으면 프로그램 내부에서도 읽을 수 없다.
    // yaml 파일에 설정된 파라미터는 계층 구조가 있을 경우 점(.) 표기법으로 가져온다.
    declare_parameter<int>("game.team_id", 29); 
    declare_parameter<int>("game.player_id", 1); // 1 | 2 | 3 | 4 | 5
    declare_parameter<string>("game.field_type", ""); // adult_size | kid_size
    declare_parameter<string>("game.player_role", ""); // striker | goal_keeper
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 4);

    // 로봇 제어 관련 파라미터 -> robot_client.cpp 에서 사용
    declare_parameter<double>("robot.vx_factor", 0.5);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.0);
    declare_parameter<double>("robot.vy_limit", 0.4);
    declare_parameter<double>("robot.vtheta_limit", 1.0);

    // 전략 관련 파라미터
    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);   // 공 인식 신뢰도 임계값

    // 카메라 관련 파라미터 
    declare_parameter<string>("vision.image_topic", "/camera/camera/color/image_raw");  // RGB 카메라 이미지 토픽
    declare_parameter<string>("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");  // 정렬된 깊이 이미지 토픽
    declare_parameter<double>("vision.cam_pixel_width", 1280);  // 카메라 가로 해상도
    declare_parameter<double>("vision.cam_pixel_height", 720);  // 카메라 세로 해상도
    declare_parameter<double>("vision.cam_fov_x", 90);  // 카메라 FOV X (도)
    declare_parameter<double>("vision.cam_fov_y", 65);  // 카메라 FOV Y (도)
    
    
    // BT
    declare_parameter<string>("tree_file_path", "");
    // 게임 컨트롤러 IP 주소
    declare_parameter<string>("game_control_ip", "0.0.0.0");
}

Brain::~Brain(){}

void Brain::init(){   
    config = std::make_shared<BrainConfig>();
    loadConfig(); // config 변수에 등록

    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    data = std::make_shared<BrainData>();
   
    tree->init();
    client->init();

    // ROS callback 연결
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    // 필드 라인 인식 결과 구독
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_vision/line_segments", SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));

}

void Brain::tick(){ tree->tick(); }

void Brain::loadConfig(){
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.treat_person_as_robot", config->treatPersonAsRobot);
    get_parameter("game.number_of_players", config->numOfPlayers);

    // 로봇 제어 관련 파라미터
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.vx_limit", config->vxLimit);
    get_parameter("robot.vy_limit", config->vyLimit);
    get_parameter("robot.vtheta_limit", config->vthetaLimit);

    // 전략 관련 파라미터 
    get_parameter("strategy.ball_confidence_threshold", config->ballConfidenceThreshold);  // 공 탐지 신뢰도 임계값

    // 카메라 관련 파라미터
    get_parameter("vision.cam_pixel_width", config->camPixX);  // 카메라 해상도 X
    get_parameter("vision.cam_pixel_height", config->camPixY);  // 카메라 해상도 Y
    double camDegX, camDegY;  // 시야각 임시 변수 선언
    get_parameter("vision.cam_fov_x", camDegX);  // FOV X (deg)
    get_parameter("vision.cam_fov_y", camDegY);  // FOV Y (deg)
    config->camAngleX = deg2rad(camDegX);  // 라디안으로 변환
    config->camAngleY = deg2rad(camDegY);  // 라디안으로 변환

    // 从视觉 config 中加载相关参数
    string visionConfigPath, visionConfigLocalPath;  // 비전 설정 파일 경로 변수 선언
    get_parameter("vision_config_path", visionConfigPath);  // 글로벌 설정 파일 경로 읽기
    get_parameter("vision_config_local_path", visionConfigLocalPath);  // 로컬 덮어쓰기 설정 파일 경로 읽기
    if (!filesystem::exists(visionConfigPath)) {  // 파일 존재 확인
        // 报错然后退出
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());  // 에러 로그 출력
        exit(1);  // 비전 설정이 없으면 프로그램 종료
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);  // YAML 파일 로드 (전역 설정)
    if (filesystem::exists(visionConfigLocalPath)) {  // 로컬 오버라이드 설정 파일이 존재하면
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);  // 로컬 YAML 로드
        MergeYAML(vConfig, vConfigLocal);  // 전역 + 로컬 설정 병합
    }
    config->camfx = vConfig["camera"]["intrin"]["fx"].as<double>();  // 카메라 내부 파라미터 fx
    config->camfy = vConfig["camera"]["intrin"]["fy"].as<double>();  // fy
    config->camcx = vConfig["camera"]["intrin"]["cx"].as<double>();  // cx
    config->camcy = vConfig["camera"]["intrin"]["cy"].as<double>();  // cy

    auto extrin = vConfig["camera"]["extrin"];  // 외부 파라미터 행렬 노드 가져오기
    for (int i = 0; i < 4; ++i) {  // 4x4 행렬 복사
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();  // YAML 값 → 행렬 요소
        }
    }
    prtDebug(format("camfx: %f, camfy: %f, camcx: %f, camcy: %f", config->camfx, config->camfy, config->camcx, config->camcy));  // 내부 파라미터 출력
    string str_cam2head = "camToHead: \n";  // 외부 파라미터 행렬 문자열로 변환
    for (int i = 0; i < 4; ++i) {  // 행 단위 반복
        for (int j = 0; j < 4; ++j) {  // 열 단위 반복
            str_cam2head += format("%.3f ", config->camToHead(i, j));  // 각 원소를 문자열로 추가
        }
        str_cam2head += "\n";  // 행 끝마다 줄바꿈
    }
    prtDebug(str_cam2head);  // camToHead 행렬 출력 (디버그용)

    // BT 관련 파라미터
    get_parameter("tree_file_path", config->treeFilePath);

    config->handle(); // 맵 관련 정보들 초기화
}


/* ----------------------------- time 관련 함수 유틸 -------------------------------*/
double Brain::msecsSince(rclcpp::Time time){
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    return (now - time).nanoseconds() / 1e6;
}

/* ------------------------- ROS Callback 관련 함수 구현 -------------------------------*/
void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg){
    data->timeLastGamecontrolMsg = get_clock()->now();
    // 处理比赛的一级状态
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // 比赛的一级状态
    vector<string> gameStateMap = {
        "INITIAL", // 초기 상태, 선수는 경기장 밖에서 대기
        "READY",   // 준비 상태, 선수는 경기장으로 진입하고 자신의 시작 위치로 이동
        "SET",     // 정지 상태, 심판기에서 경기 시작 지시가 내려올 때까지 대기
        "PLAY",    // 정상 경기 진행
        "END"      // 경기 종료
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)]; // 현재 무슨 게임 상태인지 확인
    tree->setEntry<string>("gc_game_state", gameState); // 적용

    bool isKickOffSide = (msg.kick_off_team == config->teamId); // 우리 팀이 킥오프(선공) 팀인지 여부
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide); // 적용

    // 경기의 2차(하위) 상태를 처리
    string gameSubStateType;
    switch (static_cast<int>(msg.secondary_state)) {
        case 0:
            gameSubStateType = "NONE";
            data->realGameSubState = "NONE";
            break;
        case 3:
            gameSubStateType = "TIMEOUT"; // 양 팀의 타임아웃과 심판 타임아웃을 포함
            data->realGameSubState = "TIMEOUT";
            break;

        // 현재 다른 상태는 처리하지 않고, TIMEOUT을 제외한 모든 상태를 FREE_KICK으로 처리
        case 4:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "DIRECT_FREEKICK"; // 간접 프리킥 
            data->isDirectShoot = true;
            break;
        case 5:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "INDIRECT_FREEKICK"; // 직접 프리킥
            break;
        case 6:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "PENALTY_KICK"; // 패널티킥
            data->isDirectShoot = true;
            break;
        case 7:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "CORNER_KICK"; // 코너 킥
            break;
        case 8:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "GOAL_KICK"; // 골 킥
            data->isDirectShoot = true;
            break;
        case 9:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "THROW_IN"; // 스로우 인
            break;
        default:
            gameSubStateType = "FREE_KICK";
            break;
    }
    
    // STOP: 정지; -> GET_READY: 공격 또는 수비 위치로 이동; -> SET: 대기 자세
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType); // 현재 어떤 프리킥 상황인지
    tree->setEntry<string>("gc_game_sub_state", gameSubState); // 정지, 공격 또는 수비 위치로 이동, 대기 자세인지 확인

    // 프리킥 상황에서 우리 팀이 선공(킥 수행) 팀인지 여부. 
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId); 
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;

    if (msg.teams[0].team_number == config->teamId) // teams[0]이 우리 팀인 경우 -> 팀 정보 입력
    {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    }
    else if (msg.teams[1].team_number == config->teamId) // teams[1]이 우리 팀인 경우
    {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    }
    else
    {
        // 데이터 패킷에 우리 팀 정보가 포함되어 있지 않음 → 더 이상 처리하지 않음
        prtErr(format("received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number, msg.teams[1].team_number, config->teamId));
        return;
    }

    int liveCount = 0;
    int oppoLiveCount = 0;
    // 판정(페널티) 상태 처리.
    // penalty[playerId - 1] 은 우리 팀 선수의 페널티 상태를 의미하며,
    // 페널티 상태에 있으면 움직일 수 없음
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        // 우리 팀 선수의 페널티 상태 저장
        data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);
        // 레드카드가 있으면 강제 퇴장 상태로 처리
        if (static_cast<int>(myTeamInfo.players[i].red_card_count) > 0) {
            data->penalty[i] = PENALTY_SUBSTITUTE;
        }
        // 페널티가 없는 선수면 경기 가능한 선수 수 증가
        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        // 상대 팀 선수의 페널티 상태 저장
        data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);
        // 상대 팀 선수의 레드카드가 있으면 강제 퇴장 상태로 처리
        if (static_cast<int>(oppoTeamInfo.players[i].red_card_count) > 0) {
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }
        // 상대 팀 선수의 페널티가 없는 선수면 경기 가능한 선수 수 증가
        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    // 경기 가능한 선수 수 저장
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

   
    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[config->playerId - 1] != PENALTY_NONE); // 현재 로봇이 페널티(반칙)를 받는 중인지 여부
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);

    // 로봇이 페널티를 받으면 경기장 밖으로 나갔다가 다시 들어와야 합니다. 
    // 이때 로봇의 현재 위치 정보(Odometry)가 틀어질 수 있으므로, odom_calibrated(오도메트리 보정 여부)를 false로 설정하여 위치 추정(Localization)을 초기화하겠다는 의미입니다.
    // if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); 

    // 점수 기록
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg){

    // data->camConnected = true;
    // time 관련 변수
    // time 관련 변수
    auto timePoint = detection_utils::timePointFromHeader(msg.header);
    auto now = get_clock()->now();
    data->timeLastDet = timePoint; // 디버깅 시 지연 시간 정보를 출력하기 위해 사용

    auto gameObjects = detection_utils::detectionsToGameObjects(msg, config, data); // 감지된 객체 리스트 GameObject 객체로 변환

    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++){
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);

        if (obj.label == "Goalpost")
            goalposts.push_back(obj);

        if (obj.label == "Person"){
            persons.push_back(obj);

            if (config->treatPersonAsRobot) // 사람도 로봇으로 다룰건지 
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // 객체 데이터들 전처리
    detection_utils::detectProcessBalls(balls, config, data, tree);
    // detectProcessGoalposts(goalposts);
    // detectProcessMarkings(markings);
    // detectProcessRobots(robots);

    // 处理并记录视野信息
    // detectProcessVisionBox(msg);

    // 로그 기록
    // logDetection(gameObjects);
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg){ // 필드 라인 감지 콜백
    auto timePoint = detection_utils::timePointFromHeader(msg.header); // 메시지의 타임스탬프 변환

    auto now = get_clock()->now(); // 현재 시간
    data->timeLastLineDet = timePoint; // 디버깅용 라인 감지 시각 기록

    vector<FieldLine> lines = {}; // 필드 라인 벡터 초기화
    FieldLine line; // 단일 라인 구조체 선언

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations (좌표 변환용 임시 변수)
    for (int i = 0; i < msg.coordinates.size() / 4; i++) { // 한 라인은 4개의 좌표값(x0,y0,x1,y1)으로 구성
        int index = i * 4; // 각 라인의 시작 인덱스 계산
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index]; // 시작점 좌표 (로봇, 카메라 좌표)
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1]; // y0 값
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2]; // 끝점 x1
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3]; // 끝점 y1
        detection_utils::updateLinePosToField(line, data); // 필드 좌표계로 변환
        line.timePoint = timePoint; // 감지 시점 기록
        // TODO infer line dir and id
        lines.push_back(line); // 결과 벡터에 추가
    }
    lines = detection_utils::processFieldLines(lines, config, data, tree); // 감지된 라인 병합 및 식별 처리
    data->setFieldLines(lines); // 데이터 객체에 저장
}