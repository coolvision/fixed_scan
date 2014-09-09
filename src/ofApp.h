#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

#include "ofxSimpleGuiToo.h"
#include "ofxGrabCam.h"
//#include "ofxLeapMotion.h"
#include "ofxRangeToWorld.h"


//#define SIDEBAR_WIDTH 232

#define SIDEBAR_WIDTH 450

class ofApp : public ofBaseApp {
public:

// camera position
//==============================================================================
    // set from the ui
    ofPoint target; // only x, y
    ofPoint sp; // sphercal coords posision
    float camera_tilt;

    // computed to point the camera onto the target
    ofPoint position;
    ofPoint rotation;

// saved data
//==============================================================================

    CameraOptions c_o;

    int data_step; // step for points conversion
    int vis_step; // step for the mesh & drawing

    DepthFrame avg_f; // current averaged

    bool capture_frames;
    int frame_i;
    int save_i;
    float weight_a;
    bool show_normals;

// calibration
//==============================================================================
    vector<DepthFrame *> saved_f; // snapshots (averaged)
    vector<DepthFrame *> floor_maps; // corresponding "floor maps" for calibration
    void addFloorMap(DepthFrame *f);
    void drawFloorMaps();
    void reProject();
    bool add_floor_map;
    bool clear_floor_maps;
    bool draw_correspondence;
    float floor_y;
    float floor_range;

    void clearFloorMaps();
    void drawCorrespondence();
    void findCorrespondence(); // between saved snapshots, using "floor maps"

    // manual calibration
    ofPoint pc; // position
    ofPoint rc; // rotation

    bool re_project;
//==============================================================================

    string url;

    vector<ofColor> colors;

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

    ofCamera set_arm;
    ofCamera set_kinect;
    ofCamera calibrated_kinect;


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


// Leap Motion demo
//==============================================================================
//ofxLeapMotion leap;
//vector <ofxLeapMotionSimpleHand> simpleHands;
//bool prev_grab_state;
//int frames_grabbed_n;
//ofPoint start_p;
//ofPoint curr_p;
//==============================================================================
};
