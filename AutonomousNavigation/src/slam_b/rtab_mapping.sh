roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/stereo_camera/depth/depth_registered rgb_topic:=/stereo_camera/rgb/image_rect_color camera_info_topic:=/stereo_camera/rgb/camera_info frame_id:=zed_center approx_sync:=truee visual_odometry:=false odom_topic:=/odom
