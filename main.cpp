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

#define DEGREE 3 // Grau da aproximacao polinomial
#define DEBUG 1

// TODO  - TENTAR ESTIMAR O POLINOMIO SO COM OS PONTOS QUE EXISTEM ( NAO -1 ) pra verrrr
// 		 -  NAO FAZER SLIDING WINDOW EM TODO FRAME (? como)
using namespace cv;
using namespace std;



vector<Point2f> perspectivePoints;
int perspectiveReady = 0;

// Tamanho da imagem final perspectiva
int Mapax = 500;
int Mapay = 1500;
vector<Point2f> mapa(4);

void CallBackFunc(int event, int x, int y, int flags, void* param)
{
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
	// for(int i = centerL - windowSize/2; i < centerL + windowSize/2; i++){
	// 	int sum = 0;
	// 	for (int j = 0; j < input.rows; j++){
	// 		if(input.at<uchar>(j,i) == 255){
	// 			sum++;
	// 			isZeroL = 0;
	// 		}

	// 	}
	// 	LeftLanePos.push_back(sum);
	// }

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
	// for(int i = centerR - windowSize/2; i < centerR + windowSize/2; i++){
	// 	int sum = 0;
	// 	for (int j = 0; j < input.rows; j++){
	// 		if(input.at<uchar>(j,i) == 255){
	// 			sum++;
	// 			isZeroR = 0;
	// 		}
	// 	}
	// 	RightLanePos.push_back(sum);
	// }

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
	int localMaskWidht = 70;

	Rect localMaskL;
	Rect localMaskR;
	int static ff = 1;
	int static distBetLanes;
	int static lastPeakR;
	int static lastPeakL;
	int static firstWR = input.cols*3/4;
	int offsetL = 0;
	int offsetR = 0;
//------------------------------VERIFICAR AQUI------------------------// VV
	if(1/*ff*/){
		Rect roi(0, input.rows/2, Mapax, input.rows/2);
		Mat slice(input,roi);
		Point peakPosition = findPositionHistogram(slice, input.cols/4, input.cols*3/4, input.cols/2);
		lastPeakL = peakPosition.x;
		lastPeakR = peakPosition.y;
		distBetLanes = lastPeakR - lastPeakL;
		ff = 0;
	}
	
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
			if(i == 0){
				firstWR = peakPosition.y;
			}
			offsetR += (peakPosition.y - lastPeakR);
			lastPeakR = peakPosition.y;
			localMaskR = Rect(peakPosition.y - localMaskWidht/2, 0, localMaskWidht, slice.rows);

				_XR.push_back(peakPosition.y);
			
		}else{
			if(i == 0){
				localMaskR = Rect(firstWR - localMaskWidht/2, 0, localMaskWidht, slice.rows);
				_XR.push_back(firstWR);
			}else{
				localMaskR = Rect(lastPeakL + distBetLanes - localMaskWidht/2, 0, localMaskWidht, slice.rows);
				_XR.push_back(lastPeakL + distBetLanes);
			}
				
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
		rectangle(slice, localMaskL, Scalar(0, 255, 0) , 3);
		rectangle(slice, localMaskR, Scalar(0, 255, 0) , 3);
		rectangle(slice, lane, Scalar(220, 10, 10) , 3);
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

	for(int i = 0; i < DEGREE;i++){
		cout << coeffL[i] << " ";
	}
	cout << "\n";
	for(int i = 0; i < DEGREE;i++){
		cout << coeffR[i] << " ";
	}
	cout << "\n";
  	// CALCULO DA APROX POLI

  	vector<Point> curveLaneL;
  	vector<Point> curveLaneR;
	

	for(int i = 0; i < Mapay;i++){
		
		float yL =pow(i,2)*coeffL[2] + i*coeffL[1] + coeffL[0];         
		curveLaneL.push_back(Point((int)yL,i));

		float yR =pow(i,2)*coeffR[2] + i*coeffR[1] + coeffR[0];
		curveLaneR.push_back(Point((int)yR,i));

	}

	Mat maskPoly = Mat::zeros(mask.size(),mask.type());
	Mat laneL(curveLaneL, true); 
	polylines(maskPoly, laneL, false, Scalar(255,0,0), 20, LINE_4);
	Mat laneR(curveLaneR, true);
	polylines(maskPoly, laneR, false, Scalar(255,0,0), 20, LINE_4);


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
	if(offsetR < 0 && offsetL < 0){
		if(offsetL < -30 || offsetR < -30)
			sprintf(str,"Curva Esquerda");
	}else if(offsetR > 0 && offsetL > 0){
		if(offsetL > 20 || offsetR > 20)
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

Mat extractYellow(Mat &input){

	// TECNICA POR CIELAB
	Mat output;
	cvtColor(input, output, COLOR_BGR2Lab);
	vector<Mat> channels;
	split(output,channels);
	output = channels[2].clone();
	threshold( output, output, 150, 255, THRESH_BINARY); // th na mascara amarela pq ela é cinza
	return output;
}

Mat extractWhite(Mat &input){

	//TECNICA POR CIELAB
	Mat imgHLS;
	cvtColor(input, imgHLS, COLOR_BGR2Lab);
	Mat output;
	inRange(imgHLS, Scalar(220, 100, 100), Scalar(255, 255, 255), output);
	return output;

}

Mat thresholdLane(Mat input){
	
	Mat hist;
	Mat exYellow, exWhite, border;
	Mat output;
	exYellow = exWhite =  input.clone();

	//hist = equalizeIntensity(input); //--nem precisa equalizar
	exYellow = extractYellow(input);
	exWhite = extractWhite(input);

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
   // namedWindow("FramePerspective", WINDOW_KEEPRATIO);
    //namedWindow("FrameThreshold", WINDOW_KEEPRATIO);
   	#ifdef DEBUG
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
    		//imshow( "FramePerspective", perspective );
    		thresholdFrame = thresholdLane(perspective);
    		//imshow( "FrameThreshold", thresholdFrame );
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