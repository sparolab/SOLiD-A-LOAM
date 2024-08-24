<div align="center">
  <h1>SOLiD-A-LOAM</h1>
  <a href="https://github.com/sparolab/solid/tree/master/"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/solid/tree/master/"><img src="https://img.shields.io/badge/-Linux-grey?logo=linux" /></a>
  <a href="https://sites.google.com/view/lidar-solid"><img src="https://github.com/sparolab/Joint_ID/blob/main/fig/badges/badge-website.svg" alt="Project" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://www.alphaxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/alphaXiv-2408.07330-darkred" alt="alphaXiv" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />

**[IEEE RA-L]** This repository is the official code for Narrowing your FOV with **SOLiD**: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition.

  <a href="https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko" target="_blank">Hogyun Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=wL8VdUMAAAAJ&hl=ko" target="_blank">Jiwon Choi</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=UPg-JuQAAAAJ&hl=ko" target="_blank">Taehu Sim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=9mKOLX8AAAAJ&hl=ko" target="_blank">Giseop Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko" target="_blank">Younggun Cho</a><sup>†</sup>

**[Spatial AI and Robotics Lab (SPARO)](https://sites.google.com/view/sparo/%ED%99%88?authuser=0&pli=1)**
  
  <p align="center">
    <img src="figure/kitti_05_60.gif" alt="animated" width="32%" />
    <img src="figure/kitti_05_120.gif" alt="animated" width="32%" />
    <img src="figure/kitti_05_120.gif" alt="animated" width="32%" />
  </p>

</div>

## What is SOLiD-A-LOAM?
* A real-time LiDAR SLAM package that integrates A-LOAM and ScanContext. 
    * **A-LOAM** for odometry (i.e., consecutive motion estimation)
    * **SOLiD** for coarse global localization that can deal with big drifts (i.e., place recognition as kidnapped robot problem without initial pose)
    * and iSAM2 of GTSAM is used for pose-graph optimization. 

## Prerequisites
* Ubuntu (version: 20.04)
* ceres (version: 1.14)
* gtsam (version: 4.0.0-alpha2)
  * export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
## How to use?

## Example Results

## Main Contribution
* Hogyun Kim (hg.kim@inha.edu)
* Jiwon Choi (jiwon2@inha.edu)

## Special Thanks
Thank you [Giseop Kim](https://github.com/gisbi-kim/SC-A-LOAM) for providing the base code.
