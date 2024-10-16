# Use the official ROS2 Humble base image
FROM ros:humble

# Ensure the system is up to date
RUN apt-get update && apt-get upgrade -y

# Add ROS2 repository (in case there are issues with the image)
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update

# Install build essentials, ROS2 Humble, and colcon
RUN apt-get install -y \
    python3-pip \
    build-essential \
    ros-humble-ros-base \
    python3-colcon-common-extensions

# Install behave for testing
RUN pip3 install behave

# Clean up unnecessary files
RUN rm -rf /var/lib/apt/lists/*

# Set the working directory inside the container
WORKDIR /root/ros2_ws

# Copy the ROS2 packages (format_data, system) into the container
COPY . .

# Source ROS2 and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Create an entrypoint script to automatically source ROS2 and workspace setup files
COPY ./entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
