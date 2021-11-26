#./darknet detector train cfg/drone.data cfg/yolo-drone.cfg weights/yolo-drone.weights
./darknet detector train custom_training/obj.data custom_training/yolo-drone.cfg custom_training/weights/yolo-drone.weights

export CUDA_VISIBLE_DEVICES=0
./darknet detector train custom_training/obj.data custom_training/yolo-drone.cfg custom_training/weights/yolo-drone.weights -clear -map

