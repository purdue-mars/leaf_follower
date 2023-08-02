# leaf_follower
Using GelSight and UR5e to follow a leaf

![demo-leaf-following](https://github.com/purdue-mars/leaf_follower/assets/41026849/bd875265-bcf7-4fb4-bec1-7c7bdd6bf5b4)



## Quick start
>  Requires [docker](https://docs.docker.com/get-docker/)

1. Clone repo and cd
```
git clone --recurse-submodules https://github.com/purdue-mars/leaf_follower.git
cd leaf_follower
```
2. Pull docker image `docker pull raghavauppuluri13/leaf-follower`
3. Run docker image `cd docker && ./docker-run.sh` (must be in `docker` folder when sh file is executed)
4. Run `roslaunch leaf_following_control bringup.launch` brings up dynamixel driver, gelsight, ur interface 
5. In another shell, run `cd docker && ./docker-join.sh`
6. Run `rosrun leaf_following_control controller.launch` runs following controller and grasp controller 
