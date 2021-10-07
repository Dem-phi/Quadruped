由sw导出的模型中RB_top_link的惯性值全为零，导致在gezebo中显示不出来，现已经手工添加，但是sw导出的问题未解决。
run：
    1. put the pkg into src
    2. run catkin_make
    3. source envoriment
    4. roslaunch dog_v4 dog_v4.luanch
@todo：
    1. 确定真实的质量和相关的属性，来保证仿真的准确性。
    2. 每个joint处的电机还需要调整。
    3. 加入力觉和扭矩传感器。