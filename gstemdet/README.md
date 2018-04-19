# Stem Detection - Two-Step Geometrical approach
This is a two-step geometrical approach on stem detection.

## Installation
For installation see the README of this Github-Project.

## Example
For running within ROS see *piros_stemdet*
``` bash
example [options]
```
#### Options
* `-h       ` Opens help dialogue
* `-f [path]` Path to data folder [path]. (default = ../data/)
* `-i [idx] ` Progress image number [idx] (default i = 0)
* `-p [file]` Progress image with filename [file]
* `-a       ` Progress all images
* `-d       ` Activate Drawing
* `-w       ` Activate Output to file
* `-c       ` Write CoM to HDD
* `-m [px]  ` Radius for Closing Operation (default m = 10)

For running the example retreive the binary from `devel/lib/stem/` 
#### Examples
* `example -i 10 -d` Run image #10 with draw output
* `example -a -w -c` Run all images and save center of mass to `data/CoM/`
* `example -a -w` Run all images with geometrical stem detection an write to `data/stemdec/`

## Notation for plant parts
![Legend](legend.png)