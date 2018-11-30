#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <numeric>
#include <stdio.h>
#include "polifitgsl.cpp"

#define DEGREE 3 // Grau da aproximacao polinomial + 1(?)
#define DEBUG

using namespace cv;
using namespace std;

vector<Point2f> perspectivePoints;
int perspectiveReady = 0;

// Tamanho da imagem final perspectiva
int Mapax = 200;
int Mapay = 250;
vector<Point2f> mapa(4);

void CallBackFunc(int event, int x, int y, int flags, void* param){
	Scalar color(0,100,255);
	static int i = 0;
	Mat *persMat =  (Mat*)param;
	if ( event == EVENT_FLAG_LBUTTON )
	{
		if(perspectivePoints.size() < 4){
			perspectivePoints.push_back(Point(x,y));

		}
		if(perspectivePoints.size() > 1){
			line(*persMat, perspectivePoints[i], perspectivePoints[i++%3], color, 3, 8, 0);
		}
		if(perspectivePoints.size() == 4){
			line(*persMat, perspectivePoints[3], perspectivePoints[0], color, 3, 8, 0);
			i=0;
			perspectiveReady = 1;
			cout << perspectivePoints[0] << "\n";
			cout << perspectivePoints[1] << "\n";
			cout << perspectivePoints[2] << "\n";
			cout << perspectivePoints[3] << "\n";
		}
		imshow("FrameOriginal",*persMat);
	}
}

Point findPositionHistogram(Mat input, int centerL, int centerR, int windowSize){

	// Retorna um Point no qual Point.x é o pico dentro da mascara a esquerda
	// 						  e Point.y é o pico dentro da mascara a direita

	vector<int> RightLanePos;
	vector<int> LeftLanePos;
	int isZeroR = 1;
	int isZeroL = 1;
	//---------LEFT LANE-----------//

	uint8_t* pixelPtr = (uint8_t*)input.data;
	int cn = input.channels();
	Scalar_<uint8_t> bgrPixel;

	for(int i = centerL - windowSize/2; i < centerL + windowSize/2; i++){
		int sum = 0;
		for (int j = 0; j < input.rows; j++){
			if(pixelPtr[j*input.cols + i] == 255){
				sum++;
				isZeroL = 0;
			}
		}
		LeftLanePos.push_back(sum);
	}
	//---------RIGHT LANE-----------//

	for(int i = centerR - windowSize/2; i < centerR + windowSize/2; i++){
		int sum = 0;
		for (int j = 0; j < input.rows; j++){
			if(pixelPtr[j*input.cols + i] == 255){
				sum++;
				isZeroR = 0;
			}
		}
		RightLanePos.push_back(sum);
	}

	int peakL = centerL - windowSize/2 + distance(LeftLanePos.begin(), max_element(LeftLanePos.begin(), LeftLanePos.begin() + LeftLanePos.size()));
	int peakR = centerR - windowSize/2 + distance(RightLanePos.begin(), max_element(RightLanePos.begin(), RightLanePos.begin() + RightLanePos.size()));

	if(isZeroL)
		peakL = -1;
	if(isZeroR)
		peakR = -1;

	return Point(peakL,peakR);
}



int findLane(Mat frame, Mat input, Mat &output){
	

	int subdivisions = 15; //offset "calibrado" pra 15
	int localMaskWidht = 20;
	int lineThickness = 1;
	int laneThickness = 10;

	Rect localMaskL;
	Rect localMaskR;
	int static ff = 1;
	int static distBetLanes;
	int static lastPeakR;
	int static lastPeakL;
	int offsetL = 0;
	int offsetR = 0;
//------------------------------VERIFICAR AQUI------------------------// VV
	if(1/*ff*/){
	Rect roi(0, input.rows/2, Mapax, input.rows/2);
	Mat slice(input,roi);
	Point peakPosition = findPositionHistogram(slice, input.cols/4, input.cols*3/4, input.cols/2);
	if(lastPeakL != -1)
		lastPeakL = peakPosition.x;
	else
		lastPeakL = lastPeakR - distBetLanes;
	if(lastPeakR != -1)
		lastPeakR = peakPosition.y;
	else
		lastPeakR = lastPeakL + distBetLanes;

	if(lastPeakL != -1 && lastPeakR != -1)
		distBetLanes = lastPeakR - lastPeakL;
//------------------------------------------------------------

	vector<double> _Y;
	vector<double> _XL;
	vector<double> _XR;

	Mat mask = Mat::zeros(input.size(), CV_8UC3);
	for(int i = 0; i < subdivisions; i++){

		Rect roi(0, (subdivisions-1-i)*(Mapay/subdivisions), Mapax, (Mapay/subdivisions));
		Mat slice(input,roi);

		Point peakPosition = findPositionHistogram(slice, lastPeakL, lastPeakR, localMaskWidht);


		if(peakPosition.x != -1){
			offsetL += (peakPosition.x - lastPeakL);
			lastPeakL = peakPosition.x;
			localMaskL = Rect(peakPosition.x - localMaskWidht/2, 0, localMaskWidht, slice.rows);
			_XL.push_back(peakPosition.x);

		}else{

			localMaskL = Rect(lastPeakR - distBetLanes - localMaskWidht/2, 0, localMaskWidht, slice.rows);

			_XL.push_back(lastPeakR - distBetLanes);
		}
		if(peakPosition.y != -1){

			offsetR += (peakPosition.y - lastPeakR);
			lastPeakR = peakPosition.y;
			localMaskR = Rect(peakPosition.y - localMaskWidht/2, 0, localMaskWidht, slice.rows);

			_XR.push_back(peakPosition.y);

		}else{
			localMaskR = Rect(lastPeakL + distBetLanes - localMaskWidht/2, 0, localMaskWidht, slice.rows);
			_XR.push_back(lastPeakL + distBetLanes);

		}
		_Y.push_back((subdivisions-i)*(Mapay/subdivisions));

		// Se achou as duas, a distancia entre as faixas é atualizada
		if(peakPosition.x != -1 && peakPosition.y != -1){
			distBetLanes = lastPeakR - lastPeakL;
		}

		//--DESENHA RETANGULOS SUBDIVISION--//
		#ifdef DEBUG
		Rect lane = Rect(lastPeakL + localMaskWidht/2, 0, distBetLanes - localMaskWidht, slice.rows);
		cvtColor(slice, slice, COLOR_GRAY2BGR);
		rectangle(slice, localMaskL, Scalar(0, 255, 0) , lineThickness);
		rectangle(slice, localMaskR, Scalar(0, 255, 0) ,lineThickness);
		rectangle(slice, lane, Scalar(220, 10, 10) , lineThickness);
		slice.rowRange(0, slice.rows).copyTo(mask.rowRange((subdivisions-1-i)*(Mapay/subdivisions), (subdivisions-1-i)*(Mapay/subdivisions) + (Mapay/subdivisions))); 
		#endif
	}

	double *XR = &_XR[0];
	double *XL = &_XL[0];
	double *Y = &_Y[0];
	double coeffL[DEGREE];
	double coeffR[DEGREE];

	polynomialfit(subdivisions, DEGREE, Y, XL, coeffL);
	polynomialfit(subdivisions, DEGREE, Y, XR, coeffR);

	//--Funcao alternativa pra achar os coeficientes--//
	// double crit;
	// findQuadCoefficients(Y, XL, coeffL, crit, subdivisions);
	// findQuadCoefficients(Y, XR, coeffR, crit, subdivisions);

	#ifdef DEBUG
	for(int i = 0; i < DEGREE;i++){
		cout << coeffL[i] << " ";
	}
	cout << "\n";
	for(int i = 0; i < DEGREE;i++){
		cout << coeffR[i] << " ";
	}
	cout << "\n";
	#endif
  	// CALCULO DA APROX POLI

	vector<Point> curveLaneL;
	vector<Point> curveLaneR;

	// float radiusL = 

	// cout << "RL " << radiusL << "\n";
	// cout << "RR " << radiusR << "\n";	

	for(int i = 0; i < Mapay;i++){

		float yL =pow(i,2)*coeffL[2] + i*coeffL[1] + coeffL[0];         
		curveLaneL.push_back(Point((int)yL,i));

		float yR =pow(i,2)*coeffR[2] + i*coeffR[1] + coeffR[0];
		curveLaneR.push_back(Point((int)yR,i));

	}



	Mat maskPoly = Mat::zeros(mask.size(),mask.type());
	Mat laneL(curveLaneL, true); 
	polylines(maskPoly, laneL, false, Scalar(255,0,0), laneThickness, LINE_4);
	Mat laneR(curveLaneR, true);
	polylines(maskPoly, laneR, false, Scalar(255,0,0), laneThickness, LINE_4);


	uint8_t* pixelPtr = (uint8_t*)maskPoly.data;
	int cn = mask.channels();
	Scalar_<uint8_t> bgrPixel;

	for(int i = 0; i < mask.rows; i++)
	{
		for(int j = curveLaneL[i].x ; j < curveLaneR[i].x; j++)
		{
			if(j>0 && j < mask.cols){
			pixelPtr[i*mask.cols*cn + j*cn + 0] = 0; // B
       		pixelPtr[i*mask.cols*cn + j*cn + 1] = 255; // G
       		pixelPtr[i*mask.cols*cn + j*cn + 2] = 0; // R
       	}
       }
   }


   char str[50];
	// Classifica a curva de acordo com o offset acumulado do frame// -- Pouco robusto
   int offminL = -10;
   int offminR = 10;

   if(offsetL < offminL || offsetR < offminL){
   	sprintf(str,"Curva Esquerda");

   }else if(offsetL > offminR || offsetR > offminR){
   	sprintf(str,"Curva Direita");
   }else{
   	sprintf(str,"Estrada Reta");
   }

   putText(frame, str, Point(50,100), FONT_HERSHEY_PLAIN,3, Scalar(0,255,10),3);
   sprintf(str,"Rauan Pires - 14103318");
   putText(frame, str, Point(50,50), FONT_HERSHEY_PLAIN,3, Scalar(255,50,0),3);
	//-------------------------------------------------------------//

	//---Perspectiva inversa para projetar no frame original-------//
   Mat lambda = getPerspectiveTransform( mapa, perspectivePoints );
   warpPerspective(maskPoly,output,lambda,frame.size());

   addWeighted(output, 0.5, frame, 1.0, 0.0, output);
	//-------------------------------------------------------------//
	#ifdef DEBUG
   imshow("FrameMask",mask);
   imshow("FrameMaskPoly",maskPoly);
	#endif
   return 0;
}
}

Mat calculatePerspective(vector<Point2f> perspecPoints,Mat input){

	//tl tr br bl

	Mat output;
	Mat lambda = getPerspectiveTransform( perspecPoints, mapa );

	warpPerspective(input,output,lambda,Size(Mapax,Mapay));
	return output;
}

Mat equalizeIntensity(const Mat& inputImage)
{
	if(inputImage.channels() >= 3)
	{
		Mat ycrcb;

		cvtColor(inputImage,ycrcb,COLOR_BGR2YCrCb);

		vector<Mat> channels;
		split(ycrcb,channels);

		equalizeHist(channels[0], channels[0]);

		Mat result;
		merge(channels,ycrcb);

		cvtColor(ycrcb,result,COLOR_YCrCb2BGR);

		return result;
	}
	return Mat();
}
Mat applyCLAHE(const Mat& input)
{
	Mat lab_image;
	cvtColor(input, lab_image, COLOR_BGR2Lab);

    // Extract the L channel
	vector<cv::Mat> lab_planes(3);
    split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

   // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, COLOR_Lab2BGR);

   // display the results  (you might also want to see lab_planes[0] before and after).
    return image_clahe;
}
Mat extractYellow(Mat &input){

	// TECNICA POR CIELAB
	Mat output;
	cvtColor(input, output, COLOR_BGR2Lab);
	vector<Mat> channels;
	split(output,channels);
	output = channels[2].clone();
	threshold( output, output, 165, 255, THRESH_BINARY); // th na mascara amarela pq ela é cinza

	// ACHA VERDE E SUBTRAI -- Verde parece amarelo            
	Mat outputG;
	cvtColor(input, outputG, COLOR_BGR2HSV);
	inRange(outputG, Scalar(41, 39, 64), Scalar(80, 255, 255), outputG);

	output = output - outputG;

	return output;

}

Mat extractWhite(Mat &input){

	//TECNICA POR CIELAB
	// Mat imgHLS;
	// cvtColor(input, imgHLS, COLOR_BGR2Lab);
	// Mat output;
	// inRange(imgHLS, Scalar(220, 120, 120), Scalar(255, 255, 255), output);
	// return output;
	Mat output;
	inRange(input, Scalar(220, 220, 220), Scalar(255, 255, 255), output);
	return output;

}

Mat thresholdLane(Mat input){
	
	Mat hist;
	Mat exYellow, exWhite;
	Mat output;
	exYellow = exWhite =  input.clone();
	

	exWhite = extractWhite(input);
	//input = equalizeIntensity(input); //--nem precisa equalizar
	//input = applyCLAHE(input);
	exYellow = extractYellow(input);
	#ifdef DEBUG
	imshow("Yellow",exYellow);
	imshow("White",exWhite);
	#endif

	bitwise_or(exYellow,exWhite,output);
	return output;
	//output = extractWhite(hist);
}

int main(int argc, char *argv[]){

	if(argc < 2){
		cout << "Wrong number of arguments.\n";
		cout << "./<bin> + \"video_input.mp4\"\n";
		return -1;
	}

	namedWindow("FrameOriginal", WINDOW_KEEPRATIO);
	namedWindow("FrameProcessado", WINDOW_KEEPRATIO);
	namedWindow("FramePerspective", WINDOW_KEEPRATIO);

   	#ifdef DEBUG
	namedWindow("FrameThreshold", WINDOW_KEEPRATIO);
	namedWindow("White", WINDOW_KEEPRATIO);
	namedWindow("Yellow", WINDOW_KEEPRATIO);
	namedWindow("FrameMask", WINDOW_KEEPRATIO);
	namedWindow("FrameMaskPoly", WINDOW_KEEPRATIO);
	#endif

	mapa[0] = Point2f(0,0);
	mapa[1] = Point2f(Mapax,0);
	mapa[2] = Point2f(Mapax, Mapay);
	mapa[3] = Point2f(0, Mapay);

	VideoCapture cap(argv[1]); 

	if(!cap.isOpened()){
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	Mat frame;
	Mat processed;
	Mat thresholdFrame;

	setMouseCallback("FrameOriginal", CallBackFunc, &frame);


	while(cap.isOpened()){


		cap >> frame;
		if(perspectiveReady){
			Mat perspective, thresholdFrame;
			perspective = calculatePerspective(perspectivePoints, frame);
			imshow( "FramePerspective", perspective );
			thresholdFrame = thresholdLane(perspective);
			imshow( "FrameThreshold", thresholdFrame );
			processed = Mat::zeros(frame.size(), frame.type());
			findLane(frame, thresholdFrame, processed);
			
			imshow( "FrameProcessado", processed);

		}
		imshow( "FrameOriginal", frame );



		waitKey(0);
	}
	cap.release();
	destroyAllWindows();

	return 0;

}