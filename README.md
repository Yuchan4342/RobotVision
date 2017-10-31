# RobotVision_2017F
Experiment C using OpenCV in October 2017  
Machine this program is installed into knockes down three targets which have their own markers colored red, blue and yellow.

機械はこのプログラムによって的を赤,緑,青のマーカーの順に倒します。

## Requirements 必要なもの
OpenCV

## Usage 使い方
1. Please use following command to compile:  
``$ g++ -O3 -Wno-unused-result kihon_kadai.c -o kihon_kadai `pkg-config --cflags --libs opencv` -I/usr/include/ncurses -lncurses``

2. Use following command to run. Argument number means a camera this program uses:  
`$ ./kihon_kadai 0`

## Author(Mainly in movement in while) 作者(while中の動作部分)
* Yuchan  
* Tomochi
