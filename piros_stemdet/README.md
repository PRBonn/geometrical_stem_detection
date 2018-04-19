# Stem Detection ROS Node

This package contains the ROS wrapper for the Geometrical Stem Detection library.

### Build the package

```sh
  cd catkin_ws
  catkin build piros_stemdet
```
### Run nodes with their config files

Launch the node for stem detection and echoing the stem:
`roslaunch piros_stemdet standalone.launch`

or launch with bagfile:
`roslaunch piros_stemdet example.launch`

#### Config for stem detection
To modify parameters, use the config file in ./config/stemdetection_default.yaml, which
contains the following parameters:
- `in_mask: /piros_vegsec/mask` mask topic to subscribe to
- `in_rgb: /stereo_rgb/right/image_raw_sync` image rgb topic
- `in_nir: /stereo_nir/right/image_raw_sync` image nir topic
- `out_mask: /piros_stemdet/mask` annotated mask topic which is published if `verbose = 2`
- `out_stem: /piros_stemdet/stem` published stem topic
- `closing_size: 6` Kernel size for the closing operation
- `verbose: 2` 0 = just stem topic, 1 = a few words, 2 = a whole novel

#### Config for stem echo
To modify the parameters of the visualization node, use the config file in ./config/stemecho_default.yaml:
- `text_subscriber_topic: /piros_stemdet/stem` subscribed stem topic to echo
