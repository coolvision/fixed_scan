//
//  Setup.cpp
//  kinectExample
//
//  Created by sk on 8/23/14.
//
//

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

#ifdef LEAP_DEMO
	leap.open();
#endif
    //    ofSetLogLevel(OF_LOG_VERBOSE);

    colors.push_back(ofColor::red);
    colors.push_back(ofColor::green);
    colors.push_back(ofColor::blue);
    colors.push_back(ofColor::darkRed);
    colors.push_back(ofColor::violet);
    colors.push_back(ofColor::darkGreen);
    colors.push_back(ofColor::darkBlue);


    kinect.setRegistration(true);
    kinect.init();

	kinect.open();		// opens first available kinect

	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	ofSetFrameRate(60);

	// start from the front
	bDrawPointCloud = true;

    target.set(0.0f, 0.0f, 0.0f);

    gui.addTitle("target");
	gui.addSlider("target_x", target.x, -1.0f, 1.0f);
	gui.addSlider("target_y", target.z, -1.0f, 1.0f);
    target.y = 0.0f;

    gui.addTitle("spherical");
    gui.addTitle("height");
	gui.addSlider("1", sp.x, -0.5f, 1.0f);
    gui.addTitle("radius");
    gui.addSlider("2", sp.y, 0.0f, 1.0f);
    gui.addTitle("angle");
    gui.addSlider("3", sp.z, -180, 180);
    gui.addTitle("tilt");
    gui.addSlider("4", camera_tilt, -180, 180);

    gui.addTitle("step");
    gui.addSlider("data_step", data_step, 1, 4);
    gui.addSlider("vis_step", vis_step, 1, 4);

    gui.addTitle("capture");
    gui.addToggle("show_normals", show_normals);
    gui.addSlider("weight_a", weight_a, 0.0f, 1.0f);

    gui.addTitle("calibration");
    gui.addButton("add_floor_map", add_floor_map);
    gui.addButton("clear", clear_floor_maps);
    gui.addToggle("draw_correspondence", draw_correspondence);
    gui.addSlider("floor_y", floor_y, -1.0f, 1.0f);
    gui.addSlider("floor_range", floor_range, 0.1f, 1.0f);

    gui.addTitle("parameters");
	gui.addSlider("pc_x", pc.x, -0.1f, 0.1f);
    gui.addSlider("pc_y", pc.y, -0.1f, 0.1f);
	gui.addSlider("pc_z", pc.z, -0.1f, 0.1f);
	gui.addSlider("rc_x", rc.x, -30.0f, 30.0f);
    gui.addSlider("rc_y", rc.y, -30.0f, 30.0f);
	gui.addSlider("rc_z", rc.z, -30.0f, 30.0f);

    gui.setAlignRight(false);
    gui.loadFromXML();
	gui.show();
    gui.setDefaultKeys(true);

    gui.setAutoSave(true);

    min.set(-0.5, 0, -0.5);
	max.set(0.5, 1, 0.5);


	grab_cam = new ofxGrabCam(false);
	grab_cam->reset();
	grab_cam->setPosition(1.7, 1.5, 1.5);
	grab_cam->lookAt(ofVec3f(0.0f, 0.0f, 0.0f));

#ifdef LEAP_DEMO
    prev_grab_state = false;
    frames_grabbed_n = 0;
    start_p.set(0.0f, 0.0f, 0.0f);
    curr_p.set(0,0,150);
#endif

    //    string url = "http://localhost:5000/run?";
    url = "http://asimov.dhcp.lan.london.hackspace.org.uk:5000/run?";

    //curr_f.init(kinect.getWidth(), kinect.getHeight());
    avg_f.init(kinect.getWidth(), kinect.getHeight());

    //curr_f.allocateHost();
    avg_f.allocateHost();

    if (kinect.isConnected()) {
        c_o.ref_pix_size = kinect.getZeroPlanePixelSize();
        c_o.ref_distance = kinect.getZeroPlaneDistance();
        c_o.x_resolution = kinect.getWidth();
        c_o.y_resolution = kinect.getHeight();
    }
    c_o.min[0] = -10.0f;
    c_o.min[1] = -10.0f;
    c_o.min[2] = -10.0f;
    c_o.max[0] = 10.0f;
    c_o.max[1] = 10.0f;
    c_o.max[2] = 10.0f;

    capture_frames = false;
    save_i = 0;
    frame_i = 0;
}