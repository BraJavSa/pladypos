ssh -R 8888 pladypos-usv5@10.0.10.35
labos123

cd ~/ros2usv_ws/src/pladypos
git -c http.proxy=socks5h://127.0.0.1:8888 pull