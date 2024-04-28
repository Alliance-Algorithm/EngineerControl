source /opt/ros/humble.setup.zsh
colcon build --merge-install --build-base /workspaces/Engineer_Visual/deploy/build --install-base /workspaces/Engineer_Visual/deploy/install
unison -batch /workspaces/Engineer_Visual/deploy ssh://alliance@10.147.20.5//home/alliance/deploy
ssh -t alliance@10.147.20.5 "cd deploy;docker container stop deploy_yu_heng_n_n && docker container rm deploy_yu_heng_n_n;docker run --name=deploy_yu_heng_n_n -v /dev:/dev --net=host --privileged --restart=always deploy_yu_heng_n_n"