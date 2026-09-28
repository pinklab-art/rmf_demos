# rmf_edu

경기TP 자율주행로봇 코어과정 — Open-RMF 교육용 패키지.

교재는 Confluence에 있다.
<https://pinkwink.atlassian.net/wiki/spaces/EDU/pages/3638362128>

## 패키지

| 패키지 | 내용 |
|---|---|
| `rmf_edu_maps` | Traffic Editor 맵(`.building.yaml`). 빌드할 때 Gazebo 월드와 경로망이 생성된다 |
| `rmf_edu_demos` | RMF 실행 launch, Fleet 설정, RViz 설정 |
| `rmf_edu_gz` | Gazebo 시뮬레이션 launch |
| `rmf_edu_fleet_adapter` | Nav2 로봇용 Fleet Adapter |

## 빌드

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src
git clone https://github.com/pinklab-art/rmf_demos.git
cd ~/rmf_ws
rosdep install --from-paths src -ry
colcon build --symlink-install
source ~/rmf_ws/install/setup.bash
```

`--symlink-install` 로 빌드하면 설정 파일(`rmf_edu_demos/config/`)을 고친 뒤 다시 빌드하지 않고 launch 만 다시 띄우면 된다.

## 실행

```bash
ros2 launch rmf_edu_gz workshop.launch.xml
```

RMF-Web 과 함께 쓸 때:

```bash
ros2 launch rmf_edu_gz workshop.launch.xml server_uri:="ws://localhost:8000/_internal"
```

## 이 폴더의 규칙

upstream `rmf_demos` 의 기존 파일은 고치지 않는다. 필요한 것은 `rmf_edu/` 안에 새로 넣는다.
그래야 upstream 을 다시 당길 때 충돌이 나지 않는다.
