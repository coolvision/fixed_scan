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

	leap.open();

    //    ofSetLogLevel(OF_LOG_VERBOSE);

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

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = true;

    target.set(0.0f, 0.0f, 0.0f);

    gui.addTitle("target");
	gui.addSlider("target_x", target.x, -0.700, 0.700);
	gui.addSlider("target_y", target.y, -0.700, 0.700);
//    gui.addSlider("target_z", target.z, -0.300, 0.300);

    gui.addTitle("spherical");
	gui.addSlider("h", sp.x, 0, 1.0f);
    gui.addSlider("r", sp.y, -0.5f, 1.0f);
    gui.addSlider("angle", sp.z, -180, 180);
    gui.addSlider("tilt", camera_tilt, -90, 90);


//	gui.addTitle("postition");
//	gui.addSlider("postition_x", position.x, -0.700, 0.700);
//	gui.addSlider("postition_y", position.z, -0.700, 0.700);
//    gui.addSlider("postition_z", position.y, -0.700, 0.700);
//
//	gui.addTitle("rotation");
//	gui.addSlider("rotation_z1", rotation.x, -180, 180);
//	gui.addSlider("rotation_y", rotation.y, -180, 180);
//    gui.addSlider("rotation_z2", rotation.z, -180, 180);


//    gui.addTitle("calibration");
//	gui.addSlider("x", pc.x, -0.05f, 0.05);
//    gui.addSlider("y", pc.y, -0.05f, 0.05);
//    gui.addSlider("z", pc.z, -0.05f, 0.05);
//
//	gui.addSlider("a", rc.x, -10, 10);
//    gui.addSlider("b", rc.y, -10, 10);
//    gui.addSlider("c", rc.z, -10, 10);


    gui.setAlignRight(false);
    gui.loadFromXML();
	gui.show();
    gui.setDefaultKeys(true);

    min.set(-0.5, 0, -0.5);
	max.set(0.5, 1, 0.5);

    //ofSetCoordHandedness(OF_LEFT_HANDED);

	grab_cam = new ofxGrabCam(false);
	grab_cam->reset();
	grab_cam->setPosition(1.7, 1.5, 1.5);
	grab_cam->lookAt(ofVec3f(0.0f, 0.0f, 0.0f));

    prev_grab_state = false;
    frames_grabbed_n = 0;
    start_p.set(0.0f, 0.0f, 0.0f);

    curr_p.set(0,0,150);

    //    string url = "http://localhost:5000/run?";
    url = "http://asimov.dhcp.lan.london.hackspace.org.uk:5000/run?";
}