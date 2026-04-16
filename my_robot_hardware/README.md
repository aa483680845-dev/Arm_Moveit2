-硬件组件没有太多可说的，其功能主要是对robot_dm_driver的实例化，接收ros2_controller接口命令

-[硬件组件介绍](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#hardware-components)

-这个项目是使用的jazzy版本的ros2_control,关于humbel与jazzy的区别，可以参考[这里](https://control.ros.org/jazzy/doc/ros2_control/doc/migration.html)。 
对于本项目最大的改变(这里指的是从humbel迁移到jazzy)应该就是[命令/状态接口的迁移](https://control.ros.org/jazzy/doc/ros2_control/doc/migration.html#migration-of-command-stateinterfaces)