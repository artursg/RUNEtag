# RUNETag 

**RUNETag** is a fiducial marker system which heavily relies on the robust framework of cyclic codes to offer superior occlusion resilience, accurate detection and robustness against various types of noise.

Our tags are composed by a set of circular high-contrast features (dots) spatially arranged into concentric layers. The reciprocal dot positions between layers is studied to allow the identification and pose estimation of the marker even with severe occlusions. Additionally, the marker interior is left free for any additional user payload.


For technical aspects on its inner working, please refer to:

- [CVPR paper](http://www.dsi.unive.it/~bergamasco/papers/cvpr2011-tag.pdf)
- [CVPR poster](http://www.dsi.unive.it/~bergamasco/papers/cvpr11-tag-poster.pdf)
- [Demo video](https://youtu.be/F4jdG7DJVSA)

<img src="http://www.dsi.unive.it/~bergamasco/runetag/example1.png" style="height:200px"/> 
<img src="http://www.dsi.unive.it/~bergamasco/runetag/example2.png" style="height:200px"/>
<img src="http://www.dsi.unive.it/~bergamasco/runetag/example3.png" style="height:200px"/>


## Download


Latest version of RUNEtag can be downloaded [here](http://www.dsi.unive.it/~bergamasco/runetag/RUNEtag.zip). 
Given the research nature of this work, consider it still under development. Feel free to experiment and/or improve the code for your specific application. If your are using it for research purposes, please consider citing our work.

See the license below for additional details.



## Compiling/Installing


RUNETag uses the following third-party libraries:

- [OpenCV]( http://opencv.org )
- [WinNTL]( http://www.shoup.net/ntl/doc/tour-win.html ) (Included in the bundle)
- [Boost]( http://www.boost.org ) (For RUNEtagdetect executable)
- [Qt toolkit]( http://www.qt.io ) (For RUNETagGenerator)

Additionally, to build the library you will need [CMake](www.cmake.org) version 2.8 or higher.


### Basic steps to get started with RUNETags

1. Run the CMake build system to configure your build environment
2. Compile the source code
3. Run the *codegen* executable. A file called *codes.txt* will be created in the current directory
4. Run RUNETagGenerator and open the file *codes.txt*. Tags can now be browsed and personalized
5. Once a tag is selected, save it to generate two files: ```<tagname>.pdf``` and ```<tagname>.txt```
6. Print ```<tagname>.pdf``` ensuring that no scaling occurs
7. Calibrate your camera and run RUNEtagdetect, using ```<tagname>.txt``` as model file and with the appropriate
   camera calibration parameters


## Citing


To cite our paper, please use the following:

```
@INPROCEEDINGS{Bergamasco2011, 
    author={Bergamasco, F. and Albarelli, A. and Rodola, E. and Torsello, A.}, 
    booktitle={Computer Vision and Pattern Recognition (CVPR), 2011 IEEE Conference on}, 
    title={RUNE-Tag: A high accuracy fiducial marker with strong occlusion resilience}, 
    year={2011}, 
    pages={113-120}, 
    doi={10.1109/CVPR.2011.5995544}, 
    ISSN={1063-6919}, 
    month={June}
}
```



## License

The MIT License (MIT)

Copyright (c) 2015 Filippo Bergamasco 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
