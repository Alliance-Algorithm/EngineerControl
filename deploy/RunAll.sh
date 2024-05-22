#!/bin/bash

systemctl start docker
docker restart deploy_yuheng_eye
docker restart deploy_yuheng_arm
source /opt/ros/humble/setup.sh

docker run -v /dev:/dev --network host  --privileged qzhhhi/rmcs-runtime
