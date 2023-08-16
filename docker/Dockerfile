# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:melodic-desktop-full

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN mkdir -p ~/project && \
    cd ~/project/

RUN git https://github.com/MarcFacerias/ColaborativeMPC- && \
    cd ColaborativeMPC-  && \
    git checkout release && \
    cd planner/lib/ && \

RUN pip3 install requirements.txt 
RUN ./install.sh
RUN cd ..  && cd .. && cd \ROS
RUN catkin_make 

RUN echo "ALL Done"
