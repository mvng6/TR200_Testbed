# TR200 로봇 제어를 위한 ROS Noetic + Woosh SDK 환경
FROM osrf/ros:noetic-desktop-full

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/catkin_ws

# 시스템 패키지 업데이트 및 필수 도구 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    wget \
    curl \
    vim \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치 (Woosh SDK 의존성)
RUN pip3 install --upgrade pip && pip3 install \
    "protobuf>=3.20.0,<4.0.0" \
    "websockets>=11.0,<12.0" \
    asyncio \
    "typing-extensions>=4.0.0" \
    "python-dateutil>=2.8.2" \
    "pytest>=7.0.0" \
    "pytest-asyncio>=0.18.0"

# ROS 추가 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-navigation \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-map-server \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 생성
WORKDIR $CATKIN_WS

# Catkin 워크스페이스 초기화
RUN mkdir -p $CATKIN_WS/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin_make"

# Woosh SDK 복사 및 설치
COPY src/woosh_robot_py /opt/woosh_robot_py
WORKDIR /opt/woosh_robot_py

# SDK requirements.txt 및 setup.py 수정 (호환 가능한 버전으로)
RUN sed -i 's/protobuf==4.21.0/protobuf>=3.20.0,<4.0.0/' requirements.txt && \
    sed -i 's/websockets==15.0.0/websockets>=11.0,<12.0/' requirements.txt && \
    sed -i 's/"protobuf==4.21.0"/"protobuf>=3.20.0,<4.0.0"/' setup.py && \
    sed -i 's/"websockets==15.0.0"/"websockets>=11.0,<12.0"/' setup.py

RUN pip3 install -e .

# ROS 패키지 소스 복사
COPY src/ $CATKIN_WS/src/
COPY launch/ $CATKIN_WS/launch/
COPY config/ $CATKIN_WS/config/

# 워크스페이스 빌드
WORKDIR $CATKIN_WS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 환경 설정 스크립트 복사
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 네트워크 포트 노출 (ROS 통신용)
EXPOSE 11311 11312 11313 11314 11315

# 사용자 설정 (호스트와 동일한 UID/GID 사용)
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g $GROUP_ID developer && \
    useradd -u $USER_ID -g $GROUP_ID -m -s /bin/bash developer && \
    usermod -aG sudo developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 개발자 사용자로 전환
USER developer
WORKDIR /home/developer

# ROS 환경 자동 로드 설정
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc

# 엔트리포인트 설정
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]