# Quadruped
code structure of quadruped, based on ros

#### Version 1.5

Update the gazebo model of dog 

run 

```bash
roslaunch cheetah_model gazabo.launch
```

 暂时可使用"walk" and "trot" 步态，由于PID和足端摩擦力大小的问题，"pace" and “gallop”出现打滑的问题，足端非常不稳定。

不排除是CPG自身的问题。

#### Version 2.1

完成MPC，可使用trot步态

采用多线程的形式，否则会出现接触力与步态规划不一致的情况。
