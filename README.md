# 1. 编译
```bash
# 下面指令不找到，需要下载编译工具
#sudo apt-get install python3-catkin-tools
caktin init
catkin build

```

# 2. 使用
```bash
source devel/setup.bash
roslaunch ur_description view_ur5e.launch
```