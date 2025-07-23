include .env

.build:
	docker build -t ${BASE_IMAGE} . 

.start_if_not_running:
	@if [ -z "$(shell docker ps -q -f name=jetson20noetic)" ]; then \
		echo "Container jetson20noetic is not running, starting it..."; \
		make start; \
	else \
		echo "Container jetson20noetic is already running."; \
	fi

start:
	@docker container stop jetson20noetic || true && docker container rm jetson20noetic || true
	docker run \
		--rm \
		--detach \
		-e DISPLAY=$(DISPLAY) \
		-e ROS_MASTER_URI=$(ROS_MASTER_URI) \
		-e ROS_IP=$(ROS_IP) \
		-v ./social_nav_control/:/root/catkin_ws/src/social_nav_control \
		-v ./social_nav_perception/:/root/catkin_ws/src/social_nav_perception \
		-v ./social_nav_perception/config/:/root/catkin_ws/src/zed-ros-wrapper/zed_wrapper/params/ \
		-v ./zed_resources/:/usr/local/zed/resources/ \
		-v ~/.Xauthority:/root/.Xauthority:rw \
  		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /dev:/dev \
		--network host \
		--privileged \
		--runtime=nvidia \
		--name jetson20noetic \
		${BASE_IMAGE} \
		tail -f /dev/null

debug: .start_if_not_running
	@docker exec -it jetson20noetic bash

run-perception: .start_if_not_running
	@docker exec -it jetson20noetic bash -c "source /root/catkin_ws/devel/setup.bash && roslaunch social_nav_perception perception.launch"

run-control: .start_if_not_running
	@docker exec -it jetson20noetic bash -c "source /root/catkin_ws/devel/setup.bash && rosrun social_nav_control follow_human.py"

stop:
	@docker container stop jetson20noetic || true