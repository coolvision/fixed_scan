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

// depth image segmentation (connected components)
//==============================================================================
    short *depth_data;
    int depth_data_size;
    int img_width;
    int img_height;
    uint *labels;
#define N_LABELS 1000
    ofColor label_colors[N_LABELS];
    void ConnectedComponents();
    
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
    bool show_avg;


// calibration
//==============================================================================
    vector<DepthFrame *> saved_f; // snapshots (averaged)
    vector<DepthFrame *> floor_maps; // corresponding "floor maps" for calibration

    bool opt_parameter[6];

    void addFloorMap(DepthFrame *f, bool make_mesh);

    void reProject(bool make_mesh);

    void drawFloorMaps();
    void saveMaps();
    void loadMaps();

    bool add_floor_map;
    bool clear_floor_maps;
    bool draw_correspondence;
    bool draw_plane;
    bool draw_mesh;
    float floor_y;
    float floor_range;
    bool plane_correspondence;

    float max_depth;

    void clearFloorMaps();
    float findCorrespondence(bool draw); // between saved snapshots, using "floor maps"
    float planeCorrespondence(bool draw);
    void gdStep();
    void planeGDStep();

    // manual calibration
    ofPoint pc; // position
    ofPoint rc; // rotation
    ofPoint kc;
    bool reset_parameters;

    bool re_project;

    // gradient descent
    bool run_gd;
    bool run_plane_gd;
    float gd_alpha;
    float gd_epsilon;

    float ref_pix_size;
    float ref_distance;

    float x_curr[6]; // optimization parameters
    float x_next[6]; // 2 samples to estimate gradient

    float f_curr[6];
    float f_next[6];

    float grad[6]; // gradient value

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
    ofCamera calibrated_kinect_local;


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
