unison -batch /workspaces/Engineer_Visual/deploy ssh://alliance@192.168.1.55//home/alliance/deploy
ssh -t alliance@192.168.1.55 "cd deploy/d1;
sudo docker container stop deploy_yuheng_eye && sudo docker container rm deploy_yuheng_eye;
sudo docker image rm deploy_yuheng_eye;sudo docker build --network=host -f Dockerfile -t deploy_yuheng_eye .;
cd ../d2;
sudo docker container stop deploy_yuheng_arm && sudo docker container rm deploy_yuheng_arm;
sudo docker image rm deploy_yuheng_arm;sudo docker build --network=host -f Dockerfile -t deploy_yuheng_arm ."