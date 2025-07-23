FROM nvcr.io/nvidia/l4t-tensorrt:r8.4.1.5-devel

SHELL ["/bin/bash", "-c"]

RUN rm -Rf /etc/apt/sources.list.d/cuda.list

RUN apt-get update -y && apt-get install -y git vim wget zstd

####################################################################################################
########################################### ROS NOETIC #############################################
####################################################################################################

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sed -i 's=deb https://repo.download.nvidia.com/jetson/common r35.1 main=# deb https://repo.download.nvidia.com/jetson/common r35.1 main=' /etc/apt/sources.list &&\
    apt-get update -y && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential &&\
    rosdep init && \
    rosdep update    

RUN python3 -m pip install --upgrade pip

####################################################################################################
########################################### ZED SDK 4.0 ############################################
####################################################################################################

WORKDIR /zed

RUN wget -q -O ZED_SDK_Jetson_40.run https://download.stereolabs.com/zedsdk/4.0/l4t35.4/jetsons
RUN chmod +x ZED_SDK_Jetson_40.run && \
    DEBIAN_FRONTEND=noninteractive ./ZED_SDK_Jetson_40.run -- silent && \
    rm ZED_SDK_Jetson_40.run

RUN python3 -m pip install cython numpy opencv-python pyopengl
RUN apt-get update -y && apt-get install --no-install-recommends python3-pip -y && \
    wget download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py &&  \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install numpy opencv-python *.whl && \
    rm *.whl


WORKDIR /root/catkin_ws

RUN mkdir -p /root/catkin_ws/src/ && source /opt/ros/noetic/setup.bash && catkin init && catkin build

RUN cd src && git clone  --branch v4.0.5 --recurse-submodules -j8 https://github.com/stereolabs/zed-ros-wrapper.git
RUN apt update && source devel/setup.bash && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y 
RUN python3 -m pip install -U pip && python3 -m pip install opencv-contrib-python

RUN cd src && git clone https://github.com/stereolabs/zed-ros-examples.git
RUN apt update && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . --ignore-src -r -y
RUN catkin build -DCMAKE_BUILD_TYPE=Release


RUN source devel/setup.bash && \
    catkin config --cmake-args -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" && \
    catkin build zed_wrapper zed_examples


####################################################################################################
########################################### SOCIAL NAV #############################################
####################################################################################################

COPY ./social_nav_control /root/catkin_ws/src/social_nav_control
COPY ./social_nav_perception /root/catkin_ws/src/social_nav_perception
RUN catkin build social_nav_control social_nav_perception

# Place here any additional dependencies for the Social Navigation project

CMD bash