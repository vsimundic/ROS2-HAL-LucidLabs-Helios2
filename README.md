# LucidLabs Helios2/Helios2+ ToF (Time-of-Flight) camera ROS2 HAL

## Prerequisites
Please download SDK from LucidLabs [support page](https://thinklucid.com/downloads-hub) and unpack `ArenaSDK_Linux_x64` folder somewhere in your system (good places are either `/usr/local`, `/opt`, `~/bin`).

Open `.bashrc` and add the following lines to the end:
```
export ARENA_GCCVER=54_v3_3_LUCID
export ARENA_PATH=/path/to/ArenaSDK_Linux_x64
```
The `ARENA_GCCVER` is the file suffix of the files present in `ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64`.

Finally, open a new terminal or source `.bashrc` before running `colcon`.

## Triton RGB

RGB is optional; depth-only launch behavior is unchanged. To append a PCL-compatible
packed `rgb` FLOAT32 field to `/camera/points` and publish `/camera/rgb/image_raw`
as `rgb8`, launch with absolute paths to the calibration files:

```bash
ros2 launch helios2_hal hal_lucidlabs_helios2.launch.py \
  rgb:=true \
  orientation_file:=/home/user/helios2-ros2/data/calibration/orientation.yml \
  intrinsics_file:=/home/user/helios2-ros2/data/calibration/tritoncalibration.yml
```

Use paths visible inside the container when running Docker. If more than one
Triton is connected, also set `rgb_serial:=<serial>`. The selected camera must
support color and use the calibrated full-frame resolution, without offsets,
binning or decimation. Stop ArenaView/calibration acquisition before launching.

XYZ remains in the Helios frame in metres. The existing intensity field is
retained when enabled. Projection uses the Helios-to-Triton rotation vector,
translation in millimetres, and all OpenCV distortion coefficients in
`orientation.yml`; these intrinsics must match `tritoncalibration.yml`.
No calibration file is modified.

The latest complete Triton frame is paired by host receipt time, with a default
maximum age of 200 ms (`rgb.max_age_ms` node parameter). This is not hardware
synchronization; moving scenes can exhibit color misalignment. Invalid points,
points behind the Triton or outside its image, and frames without fresh RGB get
zero (black) color. Occlusion between cameras is not resolved. Warnings report
missing frames or zero projected points. The raw RGB image uses
`triton_optical_frame` (configurable with `rgb.frame_id`).

The supplied orientation currently has a translation norm of approximately
11,145,000 mm. That is implausible for a mounted camera pair and requires checking
or repeating the orientation calibration before expecting usable alignment.
The driver warns about large translations and never substitutes an identity
transform. A live check should confirm image color, point-cloud alignment on a
stationary target, and behavior when the Triton disconnects.
