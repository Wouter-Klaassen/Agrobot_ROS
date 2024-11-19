# Agrobot
Agrotbot Gantry Challenge Software
## installation:

setup
```
source /opt/ros/humble/setup.sh
```
1. create workspace
```
mkdir example_ws/src
cd example_ws/src
```
2. clone git content

```
git clone https://github.com/Wouter-Klaassen/Agrobot.git
```
3. install dependencies
```
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
4. build workspace
```
colcon build
```
5. enjoy :)
