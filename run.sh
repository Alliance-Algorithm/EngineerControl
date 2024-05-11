apt update
apt install -y libopencv-dev
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
ln -s /usr/include/eigen3/unsupported /usr/include/unsupported
ln -s /usr/include/eigen3/Eigen /usr/include/Eigen    
ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2  
ln -s /usr/include/eigen3/signature_of_eigen3_matrix_library /usr/include/signature_of_eigen3_matrix_library          
# colcon build --merge-install && ros2 run visual_exchange visual_exchange 