# 🛣️ LaneFollower_WS

ROS Noetic 기반 **차선 인식 → 경로 생성 → Pure-Pursuit 제어** 파이프라인입니다.  
건국대학교 2025-1 *Autonomous Vehicle Platform* 과제-연구를 위해 작성한 코드와 데모를 관리합니다.  
(Matplotlib 오프라인 검증 스크립트까지 함께 포함)

---

## 📁 프로젝트 구조

lane_follower_ws/
├── src/
│ └── lane_follower/
│ ├── launch/ # 🔧 3-노드 런치 파일
│ │ └── lane_follower.launch
│ ├── scripts/ # 🐍 Python 노드
│ │ ├── lane_detector.py # ① BEV + 차선 추출
│ │ ├── path_planner.py # ② 경로/Look-ahead 계산
│ │ └── pure_pursuit_ctrl.py # ③ 조향·스로틀 출력
│ ├── CMakeLists.txt
│ └── package.xml
└── demo/
└── lane_demo_offline.py # Matplotlib 데모 (ROS 미사용)
'''

---
> `lane_demo_offline.py` 는 ROS 없이 비디오 파일을 읽어 **실시간 시각화**(차선·경로·Look-ahead·조향각 출력)를 확인할 때 사용합니다.

---

## 🚀 주요 기능

### ✅ 차선 인식 (LaneDetector)
* Bird-Eye 변환 & HLS Color Mask  
* 원형 오브젝트(신호등·표지판) 제거  
* 히스토그램 + 슬라이딩 윈도우로 좌/우 차선 픽셀 수집  
* **좌(1차)/우(3차) Poly-fit** 계수 `/lane/*_fit` 토픽 발행  
* BEV & 디버그 이미지 `/car/*_image` 토픽 발행

### ✅ 경로 생성 (PathPlanner)
* 우측 차선 기준 **차량 중심선** 함수 `P(x)` 계산  
* Look-ahead 거리 `x_la` (기본 0.85 m) 지점의 목표 좌표 산출  
* `/path/lookahead_point` 발행

### ✅ Pure-Pursuit 제어 (PurePursuitCtrl)
* Look-ahead 점으로부터 조향각 계산  
* 스로틀 0.8 (흔들림 확인용 상수) 발행  
* `/car/steering`, `/car/throttle` 토픽 사용

### ✅ 오프라인 Matplotlib 데모
* `demo/lane_demo_offline.py`  
  * 비디오 → BEVㆍ차선ㆍ경로ㆍLook-ahead를 Figure 로 실시간 출력  
  * Pure-Pursuit 조향각을 콘솔에 표시해서 **ROS 없어도 알고리즘 검증 가능**

---

## ⚙️ 사용 방법

### 🔧 빌드 (ROS Noetic)

```bash
cd ~/workspace/lane_follower_ws
catkin_make
source devel/setup.bash

