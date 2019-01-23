#!/bin/bash

# Set your own EuRoC_PATH path to run ice-ba. Use './bin/ice_ba --help' to get the explanation for all of the flags. Flags [imgs_folder] and [iba_param_path] are necessary.
# Add flag '--save_feature' to save feature message and calibration file for back-end only mode


#设置你自己的EuRoC_PATH路径来运行ice-ba。使用'./bin/ice_ba --help'获取所有标志的说明。标志[imgs_folder]和[iba_param_path]是必要的。
#添加标志'--save_feature'以保存仅后端模式的功能消息和校准文件


EuRoC_PATH=~/dataset/EuRoC

mkdir $EuRoC_PATH/result

cmd="../bin/ice_ba --imgs_folder $EuRoC_PATH/MH_01_easy --start_idx 0 --end_idx -1 --iba_param_path ../config/config_of_stereo.txt  --gba_camera_save_path $EuRoC_PATH/result/MH_01_easy.txt --stereo --save_feature"
echo $cmd
eval $cmd
