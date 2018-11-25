#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "funcs.cpp"
#include <iostream>
#include <vector>
#include <numeric>

void applyCLAHE(Mat input, Mat output){
	
	Mat lab_img;
	cvtColor(input, lab_img, COLOR_BGR2Lab);
	vector<Mat> lab_planes(3);
	split(lab_img, lab_planes);

	Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    dst.copyTo(lab_planes[0]);
    merge(lab_planes, lab_img);

    cvtColor(lab_img, output, COLOR_Lab2BGR);
}