#ifndef KINECTCALIB

#include <XnOS.h>
#include <XnCppWrapper.h>
//OpenCV
#include "cv.h"
#include "highgui.h"
//OPIRA
#include "CaptureLibrary.h"
#include "OPIRALibrary.h"
#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"
//Bullet
#include "KCRPhysicsWorld.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
//Graphics calls
#include "osg.h"
#include "leastsquaresquat.h"

//Transforms
#include "transforms.h"

using namespace std; 
using namespace xn;

// ****** Defines ******
#define SAMPLE_XML_PATH "Data/SamplesConfig.xml" // OpenNI config file

class KCRPhysicsWorld *m_world;

// OpenNI Global
Context niContext;
DepthMetaData niDepthMD;
ImageMetaData niImageMD;

bool running = true;
bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion);
void loadKinectTransform(char *filename);
osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans );
void RenderScene(IplImage *arImage, Capture *capture);
osg::Node* createVArrayFromHField(const btTransform& trans);
void GenerateHeightFieldFromDepth(IplImage* depthIm, double markerDepth);
void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize);
IplImage* piRemoveSmallObj( IplImage* img_in, int size);
void keyHandler();
float FindMarkerAffineRotation();

Capture *capture;

int counter = 0;
float *ground_grid;
float MaxHeight, MinHeight;
static const int ground_grid_size = 16384;//4096

IplImage *colourIm, *depthIm;

int count_hf_generated = 0;
int main(int argc, char* argv[]) {

	markerSize.width = -1; markerSize.height = -1;
	EnumerationErrors errors;
	switch (XnStatus rc = niContext.InitFromXmlFile(SAMPLE_XML_PATH, &errors)) {
		case XN_STATUS_OK:
			break;
		case XN_STATUS_NO_NODE_PRESENT:
			XnChar strError[1024];	errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return rc; break;
		default:
			printf("Open failed: %s\n", xnGetStatusString(rc));
			return rc;
	}

	capture = new Camera(cvSize(320,240), "camera.yml");
	osg_init(calcProjection(capture->getParameters(), capture->getDistortion(), cvSize(320,240)));

	loadKinectParams("kinect.yml", &kinectParams, &kinectDistort);
	kinectDistort =0;
	kinectParams->data.db[2]=320.0; kinectParams->data.db[5]=240.0;

	niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);

	g_depth.GetMirrorCap().SetMirror(false);
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

	kinectReg = new RegistrationOPIRA(new OCVSurf());
	kinectReg->addResizedMarker("Celica.bmp", 400);

	//physics
	m_world = new KCRPhysicsWorld();
	ground_grid = new float[ground_grid_size];

	loadKinectTransform("KinectTransform.yml");

	while (running) {
		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) {
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return rc;
		}
		g_depth.GetMetaData(niDepthMD);
		g_image.GetMetaData(niImageMD);
		

		colourIm = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourIm->imageData, niImageMD.Data(), colourIm->imageSize); cvCvtColor(colourIm, colourIm, CV_RGB2BGR);
		cvFlip(colourIm, colourIm, 1);
		depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
		memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	
		cvShowImage("Kinect View", colourIm);

		IplImage *_arImage = capture->getFrame();
		IplImage *arImage = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);
		cvResize(_arImage, arImage);
		cvWaitKey(1); keyHandler();


		if(kinectTransform) {
			if (!USE_TRIMESH_GROUND) {
				if( counter == 10) {
					inpaintDepth(&niDepthMD, true); memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);

					GenerateHeightFieldFromDepth(depthIm, AverageDepth); //Heightfield to depth
					m_world->setHeightField(ground_grid);

					osg_UpdateHeightfield(ground_grid);
					counter = 0;
				} else {
					counter++;
				}
			}
			m_world->Update();
			RenderScene(arImage, capture);
			//m_world->Update();
		}
		cvReleaseImage(&arImage); cvReleaseImage(&_arImage);
		cvReleaseImage(&depthIm); cvReleaseImage(&colourIm);

	}

	osg_uninit();

	delete m_world;//physics
	delete kinectReg;

	return 0;
}

void RenderScene(IplImage *arImage, Capture *capture) {
		float scale = 10;//8.9
		btTransform trans = m_world->getCarPose();
		btQuaternion quat = trans.getRotation();
		osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW()); 
		osg::Vec3d v = osg::Vec3d(trans.getOrigin().getX()*scale, trans.getOrigin().getY()*scale,trans.getOrigin().getZ()*scale);
		osg_render(arImage, q, v, capture->getParameters(), capture->getDistortion());
		//m_world->Update();
}


bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion) {
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==0) return false; 

	CvFileNode* fileparams;
	//Read the Camera Parameters
	fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
	*params = (CvMat*)cvRead( fs, fileparams );

	//Read the Camera Distortion 
	fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
	*distortion = (CvMat*)cvRead( fs, fileparams );
	cvReleaseFileStorage( &fs );

	return true;
}

void loadKinectTransform(char *filename) {
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs!=0) {
		CvSeq *s = cvGetFileNodeByName(fs, 0, "MarkerSize")->data.seq;
		markerSize.width = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		markerSize.height = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));

		s = cvGetFileNodeByName(fs, 0, "MarkerOrigin")->data.seq;
		marker_origin.x = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		marker_origin.y = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));

		WORLD_SCALE = cvReadRealByName(fs, 0, "WorldScale", 1);
		WORLD_ANGLE = cvReadRealByName(fs, 0, "WorldAngle", 0);
		AverageDepth = cvReadRealByName(fs, 0, "AverageDepth", 0);

		CvFileNode* fileparams = cvGetFileNodeByName( fs, NULL, "KinectTransform" );
		kinectTransform = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );

		if (niContext.WaitAnyUpdateAll() == XN_STATUS_OK) {
			//Load in the marker for registration
			osg_inittracker("Celica.bmp", 400, markerSize.width);

			m_world->setWorldDepth(AverageDepth);
			m_world->setWorldScale(WORLD_SCALE);


			g_depth.GetMetaData(niDepthMD);
			inpaintDepth(&niDepthMD, true);
			depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	

			if (!USE_TRIMESH_GROUND) {
				GenerateHeightFieldFromDepth(depthIm, AverageDepth);
				m_world->setHeightField(ground_grid);
				m_world->setMinHeight(MinHeight);
				m_world->setMaxHeight(MaxHeight);
			}
			m_world->initPhysics();
			m_world->resetScene();
		}
	}
}

osg::Vec3 asOsgVec3( const btVector3& v )  {
	return osg::Vec3( v.x(), v.y(), v.z() ); 
}  

//Convert from btConvexHullShape into osg's node
osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans )  {
	btShapeHull sh( hull );
	sh.buildHull( 0. );
	int nVerts( sh.numVertices () );
	int nIdx( sh.numIndices () ); 
	if( (nVerts <= 0) || (nIdx <= 0) )     
		return( NULL );       
	const btVector3* bVerts( sh.getVertexPointer() );
	const unsigned int* bIdx( sh.getIndexPointer() );
	osg::Vec3Array* v = new osg::Vec3Array();
	v->resize( nVerts );
	unsigned int idx;  
	for( idx = 0; idx < (unsigned int)nVerts; idx++ )        
		( *v )[ idx ] = asOsgVec3( bVerts[ idx ] );
	osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );    
	for( idx = 0; idx < (unsigned int)nIdx; idx++ )         
		deui->push_back( bIdx[ idx ] );      
	osg::Vec4Array* color = new osg::Vec4Array();     
	color->push_back( osg::Vec4( 1., 1., 1., 1. ) );     
	osg::Geometry* geom = new osg::Geometry;     
	geom->setVertexArray( v );      
	geom->setColorArray( color );      
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );      
	geom->addPrimitiveSet( deui );      
	osg::ref_ptr< osg::Geode > geode = new osg::Geode();   
	geode->addDrawable( geom );       
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	float scale = 10; //scale 8.9
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);       
	mt->setPosition(osg::Vec3d(x, y, z)); 
	mt->addChild( geode.get() );  
	return mt.release();     
}  



void GenerateHeightFieldFromDepth(IplImage* depthIm, double markerDepth) {

	float ground_depth = (float)markerDepth/10;
	IplImage *depthimg1 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_32F, 1);//problem using cvthres with 64F
	IplImage *depthimg2 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	IplImage *lowerResDepth1 = cvCreateImage(cvSize(80,60), IPL_DEPTH_32F, 1);//40,30
	IplImage *lowerResDepth2 = cvCreateImage(cvSize(128,128), IPL_DEPTH_32F, 1);//64,64
	IplImage *lowerResDepth3 = cvCreateImage(cvSize(128,128), IPL_DEPTH_32F, 1);//64,64

	cvConvertScale(depthIm, depthimg1, .1);//convert img from 16U to 32F and mm to cm
	cvResize(depthimg1, lowerResDepth1, CV_INTER_NN);//use nearest neighbor interpolation
	cvSubRS(lowerResDepth1, cvScalar(ground_depth), lowerResDepth1);
	//cvShowImage("40x30 Depth Image", lowerResDepth1);
	cvSetZero(lowerResDepth2);
	cvSetZero(lowerResDepth3);
	//printf("marker's origin x = %d y = %d \n", marker_origin.x, marker_origin.y);

	//int originX = (int)marker_origin.x /16; int originY = (int)marker_origin.y /16; //640/16->40 480/16->30
	int originX = (int)marker_origin.x /8; int originY = (int)marker_origin.y /8; 
	
	//printf("new marker's origin in40x30 x = %d y = %d \n", originX, originY);
	int originX2 = 63-originX; int originY2 = 63-originY;
	//if(originX > 23 || originY > 32) { //out of bound
	if(originX > 47 || originY > 67) { //out of bound
		//int newWidth = 63 - originX; int newHeight = 63 - originY;
		int newWidth = 127 - originX2; int newHeight = 127 - originY2;
		CvRect rect1 = cvRect(0,0,newWidth,newHeight);
		CvRect rect2 = cvRect(originX2,originY2,newWidth,newHeight);
		cvSetImageROI(lowerResDepth1,rect1);
		cvSetImageROI(lowerResDepth2,rect2);
		cvCopy(lowerResDepth1,lowerResDepth2);
		cvResetImageROI(lowerResDepth1);
		cvResetImageROI(lowerResDepth2);
		//cvShowImage("64x64 Depth Image out bound", lowerResDepth2);
		//cvShowImage("128x128 Depth Image out bound", lowerResDepth2);
	} else {
		//CvRect rect = cvRect(originX,originY,40,30);
		CvRect rect = cvRect(originX2,originY2,80,60);
		cvSetImageROI(lowerResDepth2,rect);
		cvCopy(lowerResDepth1,lowerResDepth2);
		cvResetImageROI(lowerResDepth2);
	}

	CvMat* rot_mat = cvCreateMat(2,3,CV_32FC1);
	CvPoint2D32f center = cvPoint2D32f(63, 63);
	cv2DRotationMatrix( center, WORLD_ANGLE, WORLD_SCALE, rot_mat );
	cvWarpAffine( lowerResDepth2, lowerResDepth3, rot_mat );
	cvShowImage("128x128 Depth Image", lowerResDepth3);
	MaxHeight = ground_depth-40;
	MinHeight = -MaxHeight;
	float val = 0;
	for (int i = 0; i < 128; i++) {
		for (int j = 0; j < 128; j++) {
			int index = i*128+j;
				val = CV_IMAGE_ELEM(lowerResDepth3, float, 127-i, j);

				if(val > MaxHeight)
					val = MaxHeight;
				else if(val < MinHeight)
					val = MinHeight;
				
				ground_grid[index] =  val;
		}
	}

	cvReleaseImage(&depthimg1);
	cvReleaseImage(&depthimg2);
	cvReleaseImage(&lowerResDepth1);
	cvReleaseImage(&lowerResDepth2);
	cvReleaseImage(&lowerResDepth3);
	count_hf_generated++;
}

osg::Node* createWireframeDebug() {
	osg::Vec3Array* HeightFieldPoints = new osg::Vec3Array;;

	for(int i = 0; i < 127; i++) {
		for(int j = 0; j < 127; j++) {
			float x = (float)j-64;
			float y = (float)i-64;
			float z = ground_grid[i*128+j];
			HeightFieldPoints->push_back(osg::Vec3(x, y, ground_grid[i*128+j])); 
			HeightFieldPoints->push_back(osg::Vec3(x+1, y, ground_grid[i*128+j+1]));
			HeightFieldPoints->push_back(osg::Vec3(x+1, y+1, ground_grid[(i+1)*128+j+1])); 
			HeightFieldPoints->push_back(osg::Vec3(x, y+1, ground_grid[(i+1)*128+j]));
		}
	}

	osg::Geometry* geom = new osg::Geometry(); 
	geom->setVertexArray(HeightFieldPoints); 
	geom->addPrimitiveSet(new osg::DrawArrays( GL_LINES, 0, HeightFieldPoints->size()));
	
	geom->setColorBinding(osg::Geometry::BIND_OVERALL); 

	osg::Vec4Array* col = new osg::Vec4Array(); geom->setColorArray(col); col->push_back(osg::Vec4(1,1,1,0));

	osg::ref_ptr< osg::Geode > geode = new osg::Geode(); 
	geode->addDrawable(geom);
	float scale = 10;
	float x = 0; float y = 0; float z = 0;
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(osg::Quat(0,0,0,1));       
	mt->setPosition(osg::Vec3d(x, y, z)); 
	mt->addChild( geode.get() );  
	return mt.release();    
}

IplImage* piRemoveSmallObj( IplImage* img_in, int size){    
	IplImage* img_out = cvCloneImage(img_in);
	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* contours         = NULL;      
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
    double area;   

	int num_contours = cvFindContours( img_in, storage, &contours, sizeof( CvContour ), CV_RETR_LIST );

	for(int i = 0; i < num_contours; i++) {
		area = cvContourArea( contours, CV_WHOLE_SEQ ); 
		if(fabs( area ) <= size)  {     
			cvDrawContours( img_out, contours, black, black, -1, CV_FILLED, 8 );  
		} else {
			cvDrawContours( img_out, contours, white, white, 0, CV_FILLED );
		}
		contours = contours->h_next;   
	} 

	cvReleaseMemStorage( &storage );  
	return img_out;
}

void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize) {
	IplImage *depthIm, *depthImFull;
	
	if (halfSize) {
		depthImFull = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthImFull->imageData = (char*)niDepthMD->WritableData();
		depthIm = cvCreateImage(cvSize(depthImFull->width/2.0, depthImFull->height/2.0), IPL_DEPTH_16U, 1);
		cvResize(depthImFull, depthIm, 0);
	} else {
		depthIm = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthIm->imageData = (char*)niDepthMD->WritableData();
	}
	
	IplImage *depthImMask = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	for (int y=0; y<depthIm->height; y++) {
		for (int x=0; x<depthIm->width; x++) {
			CV_IMAGE_ELEM(depthImMask, char, y, x)=CV_IMAGE_ELEM(depthIm, unsigned short,y,x)==0?255:0;
		}
	}

	IplImage *depthImMaskInv = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	cvNot(depthImMask, depthImMaskInv);

	double min, max; cvMinMaxLoc(depthIm, &min, &max, 0, 0, depthImMaskInv);
	
	IplImage *depthIm8 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	float scale = 255.0/(max-min);
	cvConvertScale(depthIm, depthIm8, scale, -(min*scale));

	IplImage *depthPaint = cvCreateImage(cvGetSize(depthIm8), IPL_DEPTH_8U, 1);
	cvInpaint(depthIm8, depthImMask, depthPaint, 3, CV_INPAINT_NS);
	
	IplImage *depthIm16 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_16U, 1);
	cvConvertScale(depthPaint, depthIm16, 1/scale, min);

	if (halfSize) {
		IplImage *depthPaintedFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_16U, 1);
		cvResize(depthIm16, depthPaintedFull,0);
		IplImage *depthImMaskFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_8U, 1);
		for (int y=0; y<depthImFull->height; y++) for (int x=0; x<depthImFull->width; x++)
			CV_IMAGE_ELEM(depthImMaskFull, char, y, x)=CV_IMAGE_ELEM(depthImFull, unsigned short,y,x)==0?255:0;
		cvCopy(depthPaintedFull, depthImFull, depthImMaskFull);
		cvReleaseImage(&depthPaintedFull); cvReleaseImage(&depthImMaskFull);
		cvReleaseImage(&depthImFull);
	} else {
		cvCopy(depthIm16, depthIm, depthImMask);
	}

	cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm16);
	cvReleaseImage(&depthPaint);
	cvReleaseImage(&depthImMask); cvReleaseImage(&depthImMaskInv);
	cvReleaseImage(&depthIm);
}


inline bool getKey(int key) { return GetAsyncKeyState(key)& 0x8000; }
void keyHandler() {
	if (getKey(VK_UP)) {
		m_world->accelerateEngine();
	} else if (getKey(VK_DOWN)) {
		m_world->decelerateEngine();
	} else {
		m_world->resetEngineForce();
	}

	if (getKey(VK_LEFT)) {
		m_world->turnEngineLeft();
	} else if (getKey(VK_RIGHT)) {
		m_world->turnEngineRight();
	} else {
		 m_world->turnReset();
	}
	
	if (getKey(VK_ESCAPE)) running = false;
	if (getKey(82)) m_world->resetScene(); //R
	if (getKey(87)) { //W
		WIREFRAME_MODE = !WIREFRAME_MODE; 
		if(WIREFRAME_MODE) {
			if (!USE_TRIMESH_GROUND) {
				//setHFNode(createVArrayFromDepth());
				
			}
				//setConvexNode(osgNodeFromBtCollisionShape(m_world->getConvexShape(), m_world->getConvexTransform()));
				//osgAddConvexNode();
		} else {
//			removeExtraNode();
		}
	}
				

	if (getKey(VK_SPACE)) {
		if (calcKinectOpenGLTransform(colourIm, depthIm, &kinectTransform)) {
			//Load in the marker for registration
			osg_inittracker("Celica.bmp", 400, markerSize.width);

			m_world->setWorldDepth(AverageDepth);
			m_world->setWorldScale(WORLD_SCALE);
			if (!USE_TRIMESH_GROUND) {
				GenerateHeightFieldFromDepth(depthIm, AverageDepth);
				m_world->setHeightField(ground_grid);
				m_world->setMinHeight(MinHeight);
				m_world->setMaxHeight(MaxHeight);
			}
			m_world->initPhysics();
			m_world->resetScene();
		} else {
			printf("Couldn't find marker, please try again!\n");
		}
	}

	if (getKey(VK_RETURN)) {
		if (kinectTransform) {
			CvFileStorage *fs = cvOpenFileStorage("KinectTransform.yml", 0, CV_STORAGE_WRITE);
			cvStartWriteStruct(fs, "MarkerSize", CV_NODE_MAP); 
				cvWriteInt(fs, "width", markerSize.width);
				cvWriteInt(fs, "height", markerSize.height);
			cvEndWriteStruct(fs);

			cvStartWriteStruct(fs, "MarkerOrigin", CV_NODE_MAP); 
				cvWriteInt(fs, "x", marker_origin.x);
				cvWriteInt(fs, "y", marker_origin.y);
			cvEndWriteStruct(fs);

			cvWriteReal(fs, "WorldScale", WORLD_SCALE);
			cvWriteReal(fs, "WorldAngle", WORLD_ANGLE);
			cvWriteReal(fs, "AverageDepth", AverageDepth);

			cvWrite(fs, "KinectTransform", kinectTransform);
			cvReleaseFileStorage( &fs );
			printf("Saved Kinect Transform\n");
		}
	}

}



#endif