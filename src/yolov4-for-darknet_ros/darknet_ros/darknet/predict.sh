IMAGE=${1:-data/image9.png}
./darknet detector test cfg/drone.data cfg/yolo-drone.cfg weights/yolo-drone.weights $IMAGE
# ./darknet detector test cfg/drone.data cfg/yolo-drone.cfg weights/yolo-drone.weights data/drone.jpg
