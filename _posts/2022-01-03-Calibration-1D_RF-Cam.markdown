---
layout: post
title:  "Extrinsic calibration of a 1D range finder and a monocular camera"
date:   2022-01-03 14:34:33 +0100
categories: calibration
---

# 1D RangeFinder - Camera calibration

from the repo: <https://github.com/RiccardoGiubilato/1d-lidar-cam-calib>
[![View 1D range finder to camera calibration on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://de.mathworks.com/matlabcentral/fileexchange/104495-1d-range-finder-to-camera-calibration)

Extrinsic calibration for a 1D lidar range finder (altimeter) and a monocular camera.
Given a set of ranges and images of a planar target with a checkerboard printouts, the pipeline returns the 5DoF extrinsics between the laser altimeter and the camera (or the 6 coordinates of an oriented vector representing the measurement direction and origin of the range finder w.r.t the camera).

<img src="/images/alti_cam_fade.png" height="150"/> <img src="/images/alti_extr.png" height="150"/>

## Usage

Just run the ```main.m``` script in the ```matlab/``` folder.
There are multiple ways to try the code out:
* use the test images and range scans provided in the folders ```images/``` and ```scans/``` (camera parameters are cached in a .mat file, which can be loaded by setting the variable in line 16)
* uncomment lines 34-40 in ```main.m``` to generate a randomized synthetic dataset

## References

If you find this code useful for your projects/research, please cite the corresponding papers:
* Giubilato, R. et al. "MiniVO: Minimalistic Range Enhanced Monocular System for Scale Correct Pose Estimation", IEEE Sensors Journal, 2020
* Giubilato, R. et al. "Scale Correct Monocular Visual Odometry Using a LiDAR Altimeter", IEEE/RSJ International Conference on Intelligent Robots and Systems, 2018

```
@ARTICLE{giubilato2020minivo,
    author={R. {Giubilato} and S. {Chiodini} and M. {Pertile} and S. {Debei}},
    journal={IEEE Sensors Journal},
    title={MiniVO: Minimalistic Range Enhanced Monocular System for Scale Correct Pose Estimation},
    year={2020},
    doi={10.1109/JSEN.2020.2978334},
    ISSN={2379-9153},
}

@INPROCEEDINGS{giubilato2018scale,
    author={R. {Giubilato} and S. {Chiodini} and M. {Pertile} and S. {Debei}},
    booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    title={Scale Correct Monocular Visual Odometry Using a LiDAR Altimeter},
    year={2018},
    pages={3694-3700},
    doi={10.1109/IROS.2018.8594096},
    ISSN={2153-0858},
    month={Oct},
}
```

### Compatibility
Tested with Matlab 2018b.
Earlier versions might be incompatible with plot/legend functions.
