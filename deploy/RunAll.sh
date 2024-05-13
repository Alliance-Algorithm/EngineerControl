#!/bin/bash

systemctl start docker
docker restart deploy_yuheng_eye
docker restart deploy_yuheng_arm
source /opt/ros/humble/setup.sh