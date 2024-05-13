FROM ros:humble

COPY startup.sh /startup.sh
COPY YuHengArm /YuHengArm
COPY Data /Data

RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list 

# RUN apt-get update && apt-get -y install \
#     usbutils && rm -rf /var/lib/apt/lists/*

RUN chmod +x /startup.sh

CMD ["/startup.sh"]