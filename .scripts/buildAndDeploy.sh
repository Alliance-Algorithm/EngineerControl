source /opt/ros/humble/setup.zsh
cd /workspaces/Engineer_Visual/src && colcon build --merge-install --build-base /workspaces/Engineer_Visual/deploy/d1/build --install-base /workspaces/Engineer_Visual/deploy/d1/install 
unison -batch /workspaces/Engineer_Visual/deploy ssh://alliance@192.168.1.55//home/alliance/deploy
ssh -t alliance@192.168.1.55 "cd deploy ; 
 docker container stop deploy_yuheng_eye ; docker container rm deploy_yuheng_eye; 
 docker run -d --name=deploy_yuheng_eye -v /dev:/dev --net=host --privileged --restart=always deploy_yuheng_eye;
 docker container stop deploy_yuheng_arm ; docker container rm deploy_yuheng_arm; 
 docker run -d --name=deploy_yuheng_arm -v /dev:/dev --net=host --privileged --restart=always deploy_yuheng_arm;
 sudo cp ./RunAll.sh /usr/bin/RunAll.sh;sudo chmod +x /usr/bin/RunAll.sh;sudo cp ./RunAll.service /etc/systemd/system/RunAll.service;
 sudo systemctl daemon-reload;sudo systemctl enable RunAll;sudo systemctl start RunAll
"