# 64210_RoboticManipulation
## Installation
```
cd 64210_RoboticManipulation
docker build -t drake .
```
## Run docker
```
docker run -it --network host --gpus all -v $(pwd):/workspace -w /workspace drake
```
