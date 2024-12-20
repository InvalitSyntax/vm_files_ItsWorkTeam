#!/bin/bash

# Копирование папки launch с заменой
cp -r ./launch /home/clover/catkin_ws/src/clover/clover/

# Копирование папки clover5 с заменой
cp -r ./models /home/clover/.gazebo/

# Копирование python-скриптов с заменой
cp -f ./box.py ./circle.py ./romb.py ./triangle.py /home/clover/catkin_ws/src/clover/clover_simulation/scripts/
chmod +x /home/clover/catkin_ws/src/clover/clover_simulation/scripts/box.py
chmod +x /home/clover/catkin_ws/src/clover/clover_simulation/scripts/circle.py
chmod +x /home/clover/catkin_ws/src/clover/clover_simulation/scripts/romb.py
chmod +x /home/clover/catkin_ws/src/clover/clover_simulation/scripts/triangle.py

# Копирование файла simulator.launch с заменой
cp -f ./simulator.launch /home/clover/catkin_ws/src/clover/clover_simulation/launch/

# Копирование файла clover_aruce.world с заменой
cp -f ./clover_aruco.world /home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/

# Копирование dae-файлов с заменой
cp -f ./triangle.dae ./dodecaedr.dae /home/clover/

# Копирование файла map_nto.txt с заменой
cp -f ./map_nto.txt /home/clover/catkin_ws/src/clover/aruco_pose/map/

echo "files have been successfully uploaded."