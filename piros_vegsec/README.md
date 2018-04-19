### piros_vegsec

Vegetation Segmentation Node for ROS.

Contains segmentation taking in RGB and NIR topics, syncronizing them, and 
outputting the mask according to the desired index

##### Usage

```sh
  cd ~/catkin_ws/src
  git clone "this_repo"
  catkin build piros_vegsec --cmake-args -DCMAKE_BUILD_TYPE=Release # to build with opts
  catkin build piros_vegsec --cmake-args -DCMAKE_BUILD_TYPE=Debug # to build without opts and with gdb info
  source ~/catkin_ws/devel/setup.bash
  roslaunch piros_vegsec piros_vegsec.launch # to run
  roslaunch piros_vegsec piros_vegsec_debug.launch # to debug with gdb
```

To modify hyperparameters, use the config file in ./config/piros_vegsec.yaml, which
looks like this:

```yaml
# inputs
in_rgb_topic: /image_rgb
in_nir_topic: /image_nir

# outputs
out_exgr: /piros_vegsec/exgr
out_ndvi: /piros_vegsec/ndvi
out_mask: /piros_vegsec/mask

# selection of index
mask_index: exgr # "exgr" or "ndvi" are the options

# hyperparam
gb_k_size: 9        # gaussian blur kernel size
morph_k_size: 5     # opening and closing structural element size
gsd: 0.33           # ground sample distance [mm/px]
min_area_blob: 50   # min area to be considered vegetation [mm2]
plot_small_obj: False   # this will make smaller blobs gray, to debug
```
