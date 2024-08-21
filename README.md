# apps说明

## obj_ranging

### 配置文件

1. ***mode***

    1. **obj_track** 默认为0即可

    2. **lane_detect** 默认为0即可

2. ***path***

    1. **obj_track** 存放目标跟踪结果的文件夹路径

    2. **lane_detect** 存放车道线检测结果的文件夹路径

    3. **lane_track** 存放车道线跟踪结果的文件夹路径

    4. **obj_ranging** 程序运行时的输出路径

3. ***show***

    1. **display_switch** 是否需要打开可视化

    2. **image_mode** 图片读取模式 0-只包含图片的文件夹 1-真值文件夹

    3. **interval** 播放间隔(ms) 0-手动播放

    4. **image_path** 图片文件夹路径

    5. **mobileye_path** mobileye文件路径,与真值文件的路径保持一致即可

