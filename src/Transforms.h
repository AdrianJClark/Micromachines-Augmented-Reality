#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <XnOS.h>
#include <XnCppWrapper.h>

using namespace xn;

CvMat *kinectParams, *kinectDistort;
CvMat *kinectTransform =0;
CvMat *kinectHomo;
Registration *kinectReg;

CvSize markerSize;

DepthGenerator g_depth;
ImageGenerator g_image;

//Tham's voodoo values
CvPoint marker_origin;
float WORLD_SCALE = 1;
float WORLD_ANGLE = 0;
float AverageDepth = 0;
static const float Image_Width =  290;

//Function Prototypes
float FindMarkerAffineRotation(CvPoint3D32f *markCorn);

bool calcKinectOpenGLTransform(IplImage *colourIm, IplImage* depthIm, CvMat** transform) {
	bool found =false;
	vector<MarkerTransform> mt = kinectReg->performRegistration(colourIm, kinectParams, kinectDistort);
	if (mt.size()>0) {
		//Find the position of the corners on the image
		CvPoint2D32f *markerCorners = (CvPoint2D32f *)malloc(4*sizeof(CvPoint2D32f));
		markerCorners[0] = cvPoint2D32f(0,0); markerCorners[1] = cvPoint2D32f(mt.at(0).marker.size.width,0); 
		markerCorners[2] = cvPoint2D32f(mt.at(0).marker.size.width,mt.at(0).marker.size.height); markerCorners[3] = cvPoint2D32f(0,mt.at(0).marker.size.height);

		CvMat mCorners = cvMat(4,1,CV_32FC2, markerCorners);
		cvPerspectiveTransform(&mCorners, &mCorners, mt.at(0).homography);

		for (int i=0; i<4; i++) {
			if (markerCorners[i].x<0 || markerCorners[i].x>depthIm->width || markerCorners[i].y<0 || markerCorners[i].y>depthIm->height) {
				for (int i=0; i<mt.size(); i++) mt.at(i).clear(); mt.clear();
				free(markerCorners);
				return false;
			}
		}

		//Find the position of the corners in the real world wrt kinect
		XnPoint3D xnCorner[4], xnNewCorner[4];
		CvPoint3D32f markCorn[4];

		for (int i=0; i<4; i++) {
			markCorn[i] = cvPoint3D32f(markerCorners[i].x, markerCorners[i].y, CV_IMAGE_ELEM(depthIm, unsigned short, (int)markerCorners[i].y, (int)markerCorners[i].x));
			xnCorner[i].X = markCorn[i].x; xnCorner[i].Y = markCorn[i].y; xnCorner[i].Z = markCorn[i].z;
		}
		marker_origin = cvPoint(xnCorner[0].X, xnCorner[0].Y);
		g_depth.ConvertProjectiveToRealWorld(4, xnCorner, xnNewCorner);

		//Calculate width and height of marker in real world
		float width1 = sqrt((xnNewCorner[0].X - xnNewCorner[1].X)*(xnNewCorner[0].X - xnNewCorner[1].X) + (xnNewCorner[0].Y - xnNewCorner[1].Y)*(xnNewCorner[0].Y - xnNewCorner[1].Y) + (xnNewCorner[0].Z - xnNewCorner[1].Z)*(xnNewCorner[0].Z - xnNewCorner[1].Z));
		float width2 = sqrt((xnNewCorner[3].X - xnNewCorner[2].X)*(xnNewCorner[3].X - xnNewCorner[2].X) + (xnNewCorner[3].Y - xnNewCorner[2].Y)*(xnNewCorner[3].Y - xnNewCorner[2].Y) + (xnNewCorner[3].Z - xnNewCorner[2].Z)*(xnNewCorner[3].Z - xnNewCorner[2].Z));
		float height1 = sqrt((xnNewCorner[3].X - xnNewCorner[0].X)*(xnNewCorner[3].X - xnNewCorner[0].X) + (xnNewCorner[3].Y - xnNewCorner[0].Y)*(xnNewCorner[3].Y - xnNewCorner[0].Y) + (xnNewCorner[3].Z - xnNewCorner[0].Z)*(xnNewCorner[3].Z - xnNewCorner[0].Z));
		float height2 = sqrt((xnNewCorner[2].X - xnNewCorner[1].X)*(xnNewCorner[2].X - xnNewCorner[1].X) + (xnNewCorner[2].Y - xnNewCorner[1].Y)*(xnNewCorner[2].Y - xnNewCorner[1].Y) + (xnNewCorner[2].Z - xnNewCorner[1].Z)*(xnNewCorner[2].Z - xnNewCorner[1].Z));
		WORLD_SCALE = Image_Width/ ((width1+width2)/2.0);
		//WORLD_SCALE = 1.2;
		float height = WORLD_SCALE*(height1+height2)/2.0;
		markerSize.width = (int) Image_Width;
		markerSize.height = (int) height;
		//markerSize.width = (int) (width1+width2)/2.0;
		//markerSize.height = (int) (height1+height2)/2.0;
		printf("Marker Size %dx%d and scale = %.2f \n", markerSize.width, markerSize.height, WORLD_SCALE);

		//Render the marker corners
		IplImage *kinectMarker = cvCloneImage(colourIm);
		for (int i=0; i<4; i++) cvCircle(kinectMarker, cvPoint(markerCorners[i].x, markerCorners[i].y), 3, cvScalar(0,0,255), -1);
		cvShowImage("Kinect Found Marker", kinectMarker);
		cvReleaseImage(&kinectMarker);

		free(markerCorners);
		//Calculate Kinect to OpenGL Transform
		{
			vector <CvPoint3D32f> srcPoints3D, dstPoints3D; vector <CvPoint2D32f> srcPoints2D, dstPoints2D;
			srcPoints2D.resize(50); srcPoints3D.resize(50); dstPoints2D.resize(50); dstPoints3D.resize(50);
			float xStep = float(markerSize.width)/9.0; float yStep = float(markerSize.height)/4.0;
			float xStep1 = float(mt.at(0).marker.size.width)/9.0; float yStep1 = float(mt.at(0).marker.size.height)/4.0;
			for (int y=0; y<5; y++) {
				for (int x=0; x<10; x++) {
					int index = x+(y*10);
					srcPoints3D.at(index) = cvPoint3D32f(x*xStep, y*yStep, 0);
					srcPoints2D.at(index) = cvPoint2D32f(x*xStep1, y*yStep1);
				}
			}
			
			CvMat mSrcCorners = cvMat(50,1,CV_32FC2, &srcPoints2D[0]); CvMat mDstCorners = cvMat(50,1,CV_32FC2, &dstPoints2D[0]);
			cvPerspectiveTransform(&mSrcCorners, &mDstCorners, mt.at(0).homography);

			XnPoint3D _xnCorner[50], _xnNewCorner[50]; 
			for (int i=0; i<50; i++) {_xnCorner[i].X = dstPoints2D[i].x; _xnCorner[i].Y = dstPoints2D[i].y; _xnCorner[i].Z = CV_IMAGE_ELEM(depthIm, unsigned short, (int)_xnCorner[i].Y, (int)_xnCorner[i].X);}
			g_depth.ConvertProjectiveToRealWorld(50, _xnCorner, _xnNewCorner);
			for (int i=0; i<50; i++) {dstPoints3D[i] = cvPoint3D32f(_xnNewCorner[i].X, _xnNewCorner[i].Y, _xnNewCorner[i].Z);}

			*transform = findTransform(dstPoints3D, srcPoints3D);
			kinectHomo = cvCloneMat(mt.at(0).homography);

			for (int y=0; y<4; y++) {
				for (int x=0; x<4; x++) {
					printf("%.2f\t", CV_MAT_ELEM((**transform), float, y,x));
				}
				printf("\n");
			}
		}
		WORLD_ANGLE = FindMarkerAffineRotation(markCorn);
		float sum=0;
		for(int i = 0; i < 4; i++) sum += markCorn[i].z;
		AverageDepth = sum/4;

		found = true;
		}
	
	for (int i=0; i<mt.size(); i++) mt.at(i).clear(); mt.clear();
	return found;
}


float FindMarkerAffineRotation(CvPoint3D32f *markCorn) {	
	//markCorn
	float x,y,magnitude,val,angle,scaleDown = 8;
	float cornerX[4], cornerY[4];
	cornerX[0] = markCorn[0].x/scaleDown; cornerY[0] = markCorn[0].y/scaleDown;
	cornerX[1] = markCorn[1].x/scaleDown; cornerY[1] = markCorn[1].y/scaleDown;
	//cornerX[2] = markCorn[2].x/scaleDown; cornerY[2] = markCorn[2].y/scaleDown;
	//cornerX[3] = markCorn[3].x/scaleDown; cornerY[3] = markCorn[3].y/scaleDown;
	x = cornerX[1]-cornerX[0];
	y = cornerY[1]-cornerY[0];
	magnitude = sqrt((float)x*x + y*y);
	val = x/magnitude;
	angle = acos(val)*180/CV_PI;//x=1,y=0
	return -angle;//opposite direction
}

#endif