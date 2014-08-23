#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

#include "ofxSimpleGuiToo.h"
#include "ofxGrabCam.h"

#include "ofxLeapMotion.h"

class ofApp : public ofBaseApp {
public:
    string url;
    ofxLeapMotion leap;
	vector <ofxLeapMotionSimpleHand> simpleHands;
    bool prev_grab_state;
    int frames_grabbed_n;
    ofPoint start_p;

    ofPoint curr_p;

	void setup();
	void update();
	void draw();
	void exit();

    bool save_points;
    vector<ofCamera *> camera_positions;
    vector<ofMesh *> meshes;
	void drawPointCloud();
    void savePoints();
    void drawCurrPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

    ofCamera camera;
    void drawCameraPose(ofxKinect *kinect,
                        ofColor color, ofMatrix4x4 transform_matrix);

    ofPoint position;
    ofPoint rotation;
    ofPoint target;
    ofPoint sp; // sphercal coords posision

    // manual calibration
    ofPoint pc; // position
    ofPoint rc; // rotation


    void drawVolume();
    ofPoint min;
	ofPoint max;
	ofxGrabCam *grab_cam;

	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	bool bDrawPointCloud;
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
