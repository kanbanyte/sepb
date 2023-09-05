# Log Project: ZED capture data
To capture data with the ZED camera we are going to use the SDK default files, and the example code found in github.

From the SDK we need:
* `ZED_Explorer`
* `ZED_SVO_Editor`

From the [ZED_EXAMPLES](https://github.com/stereolabs/zed-examples) we need:
```bash
samples\recording\export\svo\python
```

So, we will leave this folder empty.\
By default, the binary files can be found in `usr/local/bin/` on a Desktop, and in `/usr/local/zed/tools` in the Jetson Nano.

When executing these Bash scripts, the paths with `$` should be changed to the appropriate directory names, these can also be defined in a program prior to execution.

## How to Capture Data
Capturing 3 seconds and 4 frames extra. 3 seconds to settle down the auto-exposure
```bash
ZED_Explorer -r HD2K -f 15 -l 50 --cpm -m 0 -o $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP-autoexp.svo
```

Cutting the first 3 seconds due to auto-exposure
```bash
ZED_SVO_Editor -cut $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP-autoexp.svo -s 45 -e 49 $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP.svo
```

Get `.png` images
```bash
Zed_SVO_Export.py --mode 2 --input_svo_path $PATH_TO_DATA/$SCENE_NUMBER/ZED/$SCENE_NUMBER-ZED-$TIME_STAMP.svo --output_dir_path $PATH_TO_DATA/$SCENE_NUMBER/ZED/
```
