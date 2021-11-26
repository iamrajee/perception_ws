![ezgif com-gif-maker (17)](https://user-images.githubusercontent.com/25712145/143560146-e2b66055-f73d-4235-b514-88d7ba428fcb.gif)
![ezgif com-gif-maker (18)](https://user-images.githubusercontent.com/25712145/143561317-0d135220-3b17-4973-bea4-af09a3304c4c.gif)
![ezgif com-gif-maker (19)](https://user-images.githubusercontent.com/25712145/143562340-26c96ed0-a1c8-4473-9df8-dd0c6cee0762.gif)


# Run demo
python3 tools/demo_track.py video -f exps/example/mot/yolox_x_mix_det.py -c pretrained/bytetrack_x_mot17.pth.tar --fp16 --fuse --save_result --path videos/VID20210114174026_fs.mp4 --match_thresh 0.3
# convert to onnx
python3 tools/export_onnx.py --output-name bytetrack_x.onnx -f exps/example/mot/yolox_x_mix_det.py -c pretrained/bytetrack_x_mot17.pth.tar

# run onnx model
python3 onnx_inference.py --model ../../bytetrack_x.onnx
python3 onnx_inference.py --model ../../bytetrack_x.onnx  --input_shape "800,1440"

# run ByteTrack_ROS node
rosrun flocking cleaned_byteTracker.py video -f ~/perception_ws/ByteTrack/exps/example/mot/yolox_x_mix_det.py -c ~/perception_ws/ByteTrack/pretrained/bytetrack_x_mot17.pth.tar --fp16 --fuse --save_result
