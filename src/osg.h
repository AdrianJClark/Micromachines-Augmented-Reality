#ifndef OSG_H
#define OSG_H

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include "MyShadowMap.h"
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/Depth>
#include <map>
#include <sstream>

#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms\OCVSurf.h"

#include "highgui.h"
#include "main.h"

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

bool WIREFRAME_MODE = false;

void osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale);


class  CalculateBoundingBox : public osg::NodeVisitor {
public:
	CalculateBoundingBox() : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN ) {
		m_transformMatrix.makeIdentity();
	}
       
    virtual ~CalculateBoundingBox() {}
 
    virtual void apply( osg::Geode &geode ) {
        osg::BoundingBox bbox;
        for(  unsigned int i = 0; i < geode.getNumDrawables(); ++i ){
            bbox.expandBy( geode.getDrawable( i )->getBound());
        }
        osg::BoundingBox bboxTrans;
        for( unsigned int i = 0; i < 8; ++i ) {
            osg::Vec3 xvec = bbox.corner( i ) * m_transformMatrix;
            bboxTrans.expandBy( xvec );
        }
        m_boundingBox.expandBy( bboxTrans );
        traverse( geode );
    }
        
    virtual void apply( osg::MatrixTransform &node ) {
        m_transformMatrix *= node.getMatrix();
        traverse( node );
    }
        
    virtual void apply( osg::Billboard &node ){
        traverse( node );
    }
    // return the bounding box     
    osg::BoundingBox &getBoundBox() { return m_boundingBox; }  
 
protected :
    osg::BoundingBox m_boundingBox;          // the overall resultant bounding box
    osg::Matrix      m_transformMatrix;      // the current transform matrix

};

class ARTrackedNode : public osg::Group {

private:
	OPIRALibrary::Registration *r;
	typedef std::map<std::string, osg::ref_ptr<osg::MatrixTransform>> MarkerMap;
	MarkerMap mMarkers;
	osg::PositionAttitudeTransform* osgTrans;
	osg::Light* light;

public:
	ARTrackedNode(): osg::Group() {
		r = new RegistrationOPIRAMT(new OCVSurf());

		/*osg::Group* lightGroup = new osg::Group;
   
		// Create a local light.
		light = new osg::Light();
		light->setLightNum(0);
		light->setPosition(osg::Vec4(0.0f, 0.0f, 1.0, 0.0f));
		light->setAmbient(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
		light->setSpecular(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setConstantAttenuation(0.2f);
		light->setLinearAttenuation(0.05f);
		light->setQuadraticAttenuation(0.05f);

		osg::LightSource* lightSource = new osg::LightSource;	
		lightSource->setLight(light);
		lightSource->setLocalStateSetModes(osg::StateAttribute::ON); 
		lightSource->setStateSetModes(*this->getOrCreateStateSet(), osg::StateAttribute::ON);
		lightGroup->addChild(lightSource);
	
		this->addChild(lightGroup);*/
	}

	osg::Light *getLight() {
		return light;
	}

	void stop() {
		delete r;
	}
	void setVisible(int index, bool visible) {
		osg::ref_ptr<osg::Switch> s = (osg::Switch*)this->getChild(index);
		if (visible) s->setAllChildrenOn(); else s->setAllChildrenOff();
	}
	void processFrame(IplImage *mFrame, CvMat *cParams, CvMat *cDistort) {
		for (MarkerMap::iterator iter = mMarkers.begin(); iter != mMarkers.end(); iter++) {
			osg::ref_ptr<osg::Switch> s = (osg::Switch*)iter->second->getChild(0); s.get()->setAllChildrenOff();
		}
		std::vector<MarkerTransform> mTransforms = r->performRegistration(mFrame, cParams, cDistort);
		for (std::vector<MarkerTransform>::iterator iter = mTransforms.begin(); iter != mTransforms.end(); iter++) {
			MarkerMap::iterator mIter = mMarkers.find(iter->marker.name);
			if (mIter != mMarkers.end()) {
				osg::ref_ptr<osg::MatrixTransform> m = mIter->second;
				m.get()->setMatrix(osg::Matrixd(iter->transMat));
				((osg::Switch*)m->getChild(0))->setAllChildrenOn();
			}
		}
		for (int i=0; i<mTransforms.size(); i++) { 
			free(mTransforms.at(i).viewPort); free(mTransforms.at(i).projMat); 	free(mTransforms.at(i).transMat);
		}
		mTransforms.clear();
	}

	int addMarkerContent(string imageFile, int maxLengthSize, int maxLengthScale, osg::Node *node) {
		r->removeMarker(imageFile);
		if (r->addResizedScaledMarker(imageFile, maxLengthSize, maxLengthScale)) {
			Marker m = r->getMarker(imageFile);

			osgTrans = new osg::PositionAttitudeTransform();
			osgTrans->setAttitude(osg::Quat(
						osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 1.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
						));
			osgTrans->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
			osgTrans->getOrCreateStateSet()->setMode(GL_NORMALIZE, GL_TRUE);
			osgTrans->addChild(node);

			osg::ref_ptr<osg::Switch> foundObject = new osg::Switch();
			foundObject->addChild(osgTrans);

			osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
			mt->addChild(foundObject);

			osg::ref_ptr<osg::Switch> trackObject = new osg::Switch();
			trackObject->addChild(mt);
			this->addChild(trackObject);

			mMarkers[imageFile] = mt;
			return this->getChildIndex(trackObject);
		} else {
			return -1;
		}
	}

	osg::Node* getOsgTrans() {
		return osgTrans;
	}

	void addModel(osg::Node *node) {
		osgTrans->addChild(node);
	}

	void removeModel(osg::Node *node) {
		osgTrans->removeChild(node);
	}
};

/*********************************************************/
	osg::Image* mVideoImage;
	IplImage *mGLImage;
	bool captureFrame();
	osg::ref_ptr<ARTrackedNode> arTrackedNode; 
	
	//HeightField
	osg::Vec3Array* HeightFieldPoints = new osg::Vec3Array;;
	osg::Geometry* HeightFieldGeometry = new osg::Geometry(); 

	osg::ref_ptr<osg::LightSource> mLightSource;

	int celicaIndex;
	//osg::MatrixTransform* celicaTrans;
	osg::PositionAttitudeTransform * pat;
	//osg::PositionAttitudeTransform* trans;
	//osg::DrawArrays* HeightFieldArray;


	vector<CvPoint3D32f> carRect;
	CvPoint2D32f osg_carSize;
	osgViewer::Viewer viewer;


void osg_init(double *projMatrix) {
	mVideoImage = new osg::Image();
	mGLImage = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U, 3);

	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.setUpViewInWindow(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT);
	
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
	viewer.setKeyEventSetsDone(0);

	osg::ref_ptr<osg::Group> root = new osg::Group();
	viewer.setSceneData(root);

	//Create Height Field
	for (int i=0; i<127; i++) {
		for (int j=0; j<127; j++) {
			HeightFieldPoints->push_back(osg::Vec3(i-64, j-64, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-63, j-64, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-63, j-63, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-64, j-63, 0));
		}
	}

	// ----------------------------------------------------------------
	// Video background
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> bgCamera = new osg::Camera();
	bgCamera->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
	bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	bgCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_FALSE);
	bgCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, GL_FALSE);
	bgCamera->setProjectionMatrixAsOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	
	osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(osg::Vec3(0, 0, 0), osg::X_AXIS * WINDOW_WIDTH, osg::Y_AXIS * WINDOW_HEIGHT, 0, 1, 1, 0);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::Texture2D(mVideoImage));
	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geom.get());
	bgCamera->addChild(geode.get());
	root->addChild(bgCamera.get());
	
	// ----------------------------------------------------------------
	// Foreground 3D content
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> fgCamera = new osg::Camera();
	fgCamera->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	fgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
	//fgCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	fgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	fgCamera->setProjectionMatrix(osg::Matrix(projMatrix));
	root->addChild(fgCamera.get());

	arTrackedNode = new ARTrackedNode();
	fgCamera->addChild(arTrackedNode);

};


void osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale) {
	unsigned int rcvShadowMask = 0x1, castShadowMask = 0x2;

	arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());

	osg::Node *car = osgDB::readNodeFile("CelicaModel.ive");
	osg::PositionAttitudeTransform * pat0 = new osg::PositionAttitudeTransform();
	pat0->setAttitude(osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
		));
	pat0->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
	pat0->addChild(car);

	pat = new osg::PositionAttitudeTransform();
	pat->setAttitude(osg::Quat(0,0,0,1));
	pat->addChild(pat0);
	pat->setNodeMask(castShadowMask);
	//osg::Quat(1.0, osg::Vec3d(1.0, 0.0, 0.0),2.0, osg::Vec3d(0.0, 1.0, 0.0),0.0, osg::Vec3d(0.0, 0.0, 1.0)); 
	//osg::DegreesToRadians()

	osg::Light* light = new osg::Light(1);	
		light->setPosition(osg::Vec4(0, 0, 100, 1.0f));
		light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		light->setSpecular(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		light->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

		light->setConstantAttenuation(1);
		light->setLinearAttenuation(0);
		light->setQuadraticAttenuation(0);

	mLightSource = new osg::LightSource();
	mLightSource->setLight(light);
    

	osg::ref_ptr<osgShadow::MyShadowMap> sm = new osgShadow::MyShadowMap;
    sm->setTextureSize( osg::Vec2s(1024, 1024) );
	sm->setLight(mLightSource.get());
	sm->setTextureUnit( 1 );

	osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene;
	shadowedScene = new osgShadow::ShadowedScene;
	shadowedScene->setShadowTechnique( sm.get() );
	shadowedScene->setReceivesShadowTraversalMask( rcvShadowMask );
	shadowedScene->setCastsShadowTraversalMask( castShadowMask );
	shadowedScene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	// Set shadow node
	shadowedScene->addChild( pat );
	celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, shadowedScene);
	arTrackedNode->setVisible(celicaIndex, true);
	
	mLightSource->setLocalStateSetModes(osg::StateAttribute::ON); 
	mLightSource->setStateSetModes(*(shadowedScene->getOrCreateStateSet()), osg::StateAttribute::ON);
	arTrackedNode->addModel( mLightSource.get() );

	{
		// Create the Heightfield Geometry
		HeightFieldGeometry->setVertexArray(HeightFieldPoints); 
		HeightFieldGeometry->addPrimitiveSet(new osg::DrawArrays( GL_QUADS, 0, HeightFieldPoints->size()));
		HeightFieldGeometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		//Set the Heightfield to be alpha invisible
		HeightFieldGeometry->setColorBinding(osg::Geometry::BIND_OVERALL); 
		osg::Vec4Array* col = new osg::Vec4Array(); HeightFieldGeometry->setColorArray(col); col->push_back(osg::Vec4(1,1,1,0.0));

		//Create the containing geode
		osg::ref_ptr< osg::Geode > geode = new osg::Geode(); geode->addDrawable(HeightFieldGeometry);

		//Create the containing transform
		float scale = 10; float x = 0; float y = 0; float z = 0;
		osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
		mt->setScale(osg::Vec3d(scale,scale,scale));
		mt->setAttitude(osg::Quat(0,0,0,1));       
		mt->setPosition(osg::Vec3d(x, y, z)); 
		mt->addChild( geode.get() );  

		//Set up the depth testing for the landscale
		osg::Depth * depth = new osg::Depth();
		depth->setWriteMask(true); depth->setFunction(osg::Depth::Function::LEQUAL);
		mt->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

		//Set up the shadow masks
		mt->setNodeMask( rcvShadowMask );
		mt->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
		pat->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
		shadowedScene->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

		//At the heightmap twice, once for shadowing and once for occlusion
		arTrackedNode->addModel(mt);
		shadowedScene->addChild(mt);
	}


    CalculateBoundingBox bbox ;
	arTrackedNode->accept(bbox);
	osg::BoundingBox boxExtents = bbox.getBoundBox();
	osg_carSize.x = boxExtents.xMax()-boxExtents.xMin(); osg_carSize.y = boxExtents.yMax()-boxExtents.yMin(); 
	printf("x: %.2f\ty: %.2f\n", osg_carSize.x, osg_carSize.y);

	carRect.clear();
	carRect.push_back(cvPoint3D32f(-osg_carSize.x/2.0,-osg_carSize.y/2.0,0));
	carRect.push_back(cvPoint3D32f( osg_carSize.x/2.0,-osg_carSize.y/2.0,0));
	carRect.push_back(cvPoint3D32f( osg_carSize.x/2.0, osg_carSize.y/2.0,0));
	carRect.push_back(cvPoint3D32f(-osg_carSize.x/2.0, osg_carSize.y/2.0,0));

}


void osg_render(IplImage *newFrame, osg::Quat q,osg::Vec3d  v, CvMat *cParams, CvMat *cDistort) {
	cvResize(newFrame, mGLImage);
	cvCvtColor(mGLImage, mGLImage, CV_BGR2RGB);
	mVideoImage->setImage(mGLImage->width, mGLImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)mGLImage->imageData, osg::Image::NO_DELETE);
	//printf("1:%.2f  %.2f  %.2f \n", v.x(), v.y(), v.z());
	if(pat) {
		pat->setAttitude(q);
		pat->setPosition(v);
	}
	if (!viewer.done()) {
		arTrackedNode->processFrame(newFrame, cParams, cDistort);
		viewer.frame();
	}
}

void osg_uninit() {
	arTrackedNode->stop();
	cvReleaseImage(&mGLImage);
}

void osg_UpdateHeightfield(float *ground_grid) {
	int index =0;
	for(int i = 0; i < 127; i++) {
		for(int j = 0; j < 127; j++) {
			float x = (float)j-64;
			float y = (float)i-64;
			float z = ground_grid[i*128+j];
			HeightFieldPoints->at(index++) = osg::Vec3(x, y, ground_grid[i*128+j]); 
			HeightFieldPoints->at(index++) = osg::Vec3(x+1, y, ground_grid[i*128+j+1]);
			HeightFieldPoints->at(index++) = osg::Vec3(x+1, y+1, ground_grid[(i+1)*128+j+1]); 
			HeightFieldPoints->at(index++) = osg::Vec3(x, y+1, ground_grid[(i+1)*128+j]);
		}
	}
	HeightFieldGeometry->dirtyDisplayList();
}

#endif
