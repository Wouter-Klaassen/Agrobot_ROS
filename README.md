# Agrobot
Agrotbot Gantry Challenge Software
## installation:

setup
```
source /opt/ros/humble/setup.sh
```
1. create workspace
```
mkdir example_ws
cd example_ws
```
2. clone git content

```
git clone https://github.com/Wouter-Klaassen/Agrobot.git
```
3. install dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
4. build workspace
```
colcon build
```
5. enjoy :)


## develop (Not available for non-contributers):

1. create workspace
```
mkdir example_ws
cd example_ws
```
2. initialize git
```
git init
```
3. add repository
```
git remote add origin https://github.com/Wouter-Klaassen/Agrobot.git
git pull
git checkout main
```
4. switch to your new branch
```
git checkout -b your/branch
```
5. push new content
```
git add .
git commit -m your_message
git push
```
### Recommendations:
- use a github gui such as gitkraken, github desktop
