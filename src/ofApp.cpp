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

    gui.addTitle("target");
	gui.addSlider("target_x", target.x, -0.700, 0.700);
	gui.addSlider("target_y", target.y, -0.700, 0.700);
    gui.addSlider("target_z", target.z, -0.300, 0.300);

    gui.addTitle("spherical");
	gui.addSlider("h", sp.x, 0, 1.0f);
    gui.addSlider("r", sp.y, 0, 1.0f);
    gui.addSlider("angle", sp.z, -180, 180);

	gui.addTitle("postition");
	gui.addSlider("postition_x", position.x, -0.700, 0.700);
	gui.addSlider("postition_y", position.z, -0.700, 0.700);
    gui.addSlider("postition_z", position.y, -0.700, 0.700);

	gui.addTitle("rotation");
	gui.addSlider("rotation_z1", rotation.x, -180, 180);
	gui.addSlider("rotation_y", rotation.y, -180, 180);
    gui.addSlider("rotation_z2", rotation.z, -180, 180);


    gui.addTitle("calibration");
	gui.addSlider("x", pc.x, -0.05f, 0.05);
    gui.addSlider("y", pc.y, -0.05f, 0.05);
    gui.addSlider("z", pc.z, -0.05f, 0.05);

	gui.addSlider("a", rc.x, -10, 10);
    gui.addSlider("b", rc.y, -10, 10);
    gui.addSlider("c", rc.z, -10, 10);


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

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {



	}

//	vector <Hand> hands = leap.getLeapHands();
//	if( leap.isFrameNew() && hands.size() ){
//        if (hands.size() == 1) {
//            Hand &h = hands[0];
//
//            if (h.grabStrength() > 0.9) {
//                //cout << "grab " << frames_grabbed_n << " " << h.grabStrength() << endl;
//
//                if (prev_grab_state == false) {
//                    start_p.set(h.palmPosition().x, h.palmPosition().y,
//                                h.palmPosition().z);
//                }
//
//                frames_grabbed_n++;
//                prev_grab_state = true;
//            } else {
//                //cout << "no grab " << h.grabStrength() << endl;
//
//                if (prev_grab_state == true && frames_grabbed_n > 10) {
//                    ofPoint end_p(h.palmPosition().x, h.palmPosition().y,
//                                h.palmPosition().z);
//                    ofPoint move_v = end_p - start_p;
//
//                    cout << "MOVE!" << move_v.x << " " << move_v.y
//                                        << " " << move_v.z << endl;
//
//                    curr_p += move_v;
//                    ofLoadURL(url +
//                              "x=" + ofToString(curr_p.z) + "&" +
//                              "y=" + ofToString(curr_p.x) + "&" +
//                              "z=" + ofToString(curr_p.y) + "&" +
//                              "yaw=" + ofToString(0) + "&" +
//                              "pitch=" + ofToString(90) + "&" +
//                              "roll=" + ofToString(180));
//                }
//
//                frames_grabbed_n = 0;
//                prev_grab_state = false;
//            }
//
//        }
//    }

}

#define SIDEBAR_WIDTH 232

void ofApp::drawVolume() {

	float width = max.x - min.x;
	float height = max.y - min.y;

	ofPoint near_v[4];
	ofPoint far_v[4];
	near_v[0] = min;
	near_v[1].set(min.x + width, min.y, min.z);
	near_v[2].set(min.x + width, min.y + height, min.z);
	near_v[3].set(min.x, min.y + height, min.z);

	far_v[0].set(max.x - width, max.y - height, max.z);
	far_v[1].set(max.x, max.y - height, max.z);
	far_v[2] = max;
	far_v[3].set(max.x - width, max.y, max.z);

	ofSetColor(ofColor::green);
	ofSetLineWidth(1.0);
	ofLine(near_v[0], near_v[1]);
	ofLine(near_v[1], near_v[2]);
	ofLine(near_v[2], near_v[3]);
	ofLine(near_v[3], near_v[0]);

	ofLine(far_v[0], far_v[1]);
	ofLine(far_v[1], far_v[2]);
	ofLine(far_v[2], far_v[3]);
	ofLine(far_v[3], far_v[0]);

	for (int i = 0; i < 4; i++) {
		ofLine(near_v[i], far_v[i]);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);

    ofViewport(SIDEBAR_WIDTH, 0, ofGetWindowWidth() - SIDEBAR_WIDTH,
               ofGetWindowHeight());
    grab_cam->begin();


    ofDrawAxis(500);


    ofDrawGrid(1.0f, 8.0f, false, false, true, false);

    drawVolume();

    // compute absolute position and rotation
//    position.y = sp.x;
//    position.x = sp.y;
//    position.z = 0;
//    position.rotate(sp.z, ofPoint(0.0f, 1.0f, 0.0f));
//    position += target;


//    rotation.x = sp.z;
//    rotation.y = 180 + atan(sp.y / sp.x) * RAD_TO_DEG;
//    rotation.z = 180.0f;


//    rotation.x = sp.z;
//    rotation.y = 90 + atan(sp.y / sp.x) * RAD_TO_DEG;
//    rotation.z = 0.0f;


    camera.resetTransform();
    camera.pan(rotation.x);
    camera.roll(-rotation.y);
    camera.pan(rotation.z);
    camera.move(position.x, position.y, position.z);

    ofSetColor(ofColor::white);
    ofDrawSphere(target, 0.005f);
    ofLine(position, target);




    camera.transformGL();
    ofSetColor(ofColor::black);
    ofDrawBox(0.1);
	ofDrawAxis(0.2);
	camera.restoreTransformGL();

    //camera.roll(-90.0f);
    //camera.tilt(-90.0f);

    camera.move(pc);
    camera.pan(rc.x);
    camera.tilt(rc.y);
    camera.roll(rc.z);

    if (save_points) {
        savePoints();
        save_points = false;
    }
    drawPointCloud();

    camera.transformGL();
    drawCurrPointCloud();
	camera.restoreTransformGL();


    grab_cam->end();

	ofViewport(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

    if(!bDrawPointCloud) {
        kinect.drawDepth(SIDEBAR_WIDTH, 100, 400, 300);
        //kinect.draw(SIDEBAR_WIDTH, 410, 400, 300);
    }
    ofSetColor(ofColor::gray);
    ofRect(0, 0, SIDEBAR_WIDTH, ofGetWindowHeight());
    gui.draw();
}

void ofApp::drawPointCloud() {

	int w = 640;
	int h = 480;

	glPointSize(3);
	ofPushMatrix();
	ofEnableDepthTest();

    for (int i = 0; i < meshes.size(); i++) {
        camera_positions[i]->transformGL();
        meshes[i]->drawVertices();
        camera_positions[i]->restoreTransformGL();
    }

	ofDisableDepthTest();
	ofPopMatrix();
}

void ofApp::drawCurrPointCloud() {

	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
                ofPoint p = kinect.getWorldCoordinateAt(x, y);
                p /= 1000.0f;

                ofPoint world_p;
                world_p.set(p.z, p.y, -p.x);
				mesh.addVertex(world_p);
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	//ofScale(1, -1, -1);
	//ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

void ofApp::savePoints() {

    meshes.push_back(new ofMesh());
    ofMesh *mesh = meshes.back();

    camera_positions.push_back(new ofCamera());
    ofCamera *cp = camera_positions.back();
    *cp = camera;

	int w = 640;
	int h = 480;

	mesh->setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh->addColor(kinect.getColorAt(x,y));
                ofPoint p = kinect.getWorldCoordinateAt(x, y);
                p /= 1000.0f;
                ofPoint world_p;
                world_p.set(p.z, p.y, -p.x);
				mesh->addVertex(world_p);
			}
		}
	}
}


//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {

	switch (key) {

		case's':
            // send off the location
            // use coordinate system of the arm (check)

            ofLoadURL(url +
                      "x=" + ofToString(position.x * 1000.0f) + "&" +
                      "y=" + ofToString(-position.z * 1000.0f) + "&" +
                      "z=" + ofToString(position.y * 1000.0f) + "&" +
                      "yaw=" + ofToString(rotation.x) + "&" +
                      "pitch=" + ofToString(rotation.y) + "&" +
                      "roll=" + ofToString(rotation.z));
            break;

		case'r':

            // reset position to default
            ofLoadURL(url +
                      "x=" + ofToString(0) + "&" +
                      "y=" + ofToString(0) + "&" +
                      "z=" + ofToString(150) + "&" +
                      "yaw=" + ofToString(0) + "&" +
                      "pitch=" + ofToString(90) + "&" +
                      "roll=" + ofToString(180));
            // TRANS(500,0,150,yaw,pitch,roll)
            curr_p.set(0,0,150);

            break;

		case'f':
			save_points = true;
			break;

		case'n':
            pc.set(0.0f, 0.0f, 0.0f);
            rc.set(0.0f, 0.0f, 0.0f);
			break;

		case'm':
			meshes.clear();
            camera_positions.clear();
			break;

		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}

void ofApp::drawCameraPose(ofxKinect *kinect,
                    ofColor color, ofMatrix4x4 transform_matrix) {

	ofPoint near[4];
	ofPoint far[4];
	ofPoint camera_near[4];
	ofPoint camera_far[4];
	ofPoint world_near[4];
	ofPoint world_far[4];

	int width = kinect->getDepthPixelsRef().getWidth();
	int height = kinect->getDepthPixelsRef().getHeight();

	// so, there are some points for display of the camera pose
	near[0].set(0, 0, 0.0f);
	near[1].set(0, height, 0.0f);
	near[2].set(width, height, 0.0f);
	near[3].set(width, 0, 0.0f);
	far[0].set(0, 0, -1500);
	far[1].set(0, height, -1500);
	far[2].set(width, height, -1500);
	far[3].set(width, 0, -1500);

	// first, transform some points into camera coordinates
	for (int i = 0; i < 4; i++) {
		camera_near[i] =
        kinect->getWorldCoordinateAt(near[i].x, near[i].y, near[i].z);
		camera_far[i] =
        kinect->getWorldCoordinateAt(far[i].x, far[i].y, far[i].z);

		camera_near[i] /= 1000.0;
		camera_far[i] /= 1000.0;
	}

	ofSetLineWidth(1.0);

	// now transform this points
	for (int i = 0; i < 4; i++) {
		world_near[i] = camera_near[i] * transform_matrix;
		world_far[i] = camera_far[i] * transform_matrix;
	}
	ofSetColor(color);
	for (int i = 0; i < 4; i++) {
		ofLine(world_near[i], world_far[i]);
	}
	ofLine(world_far[0], world_far[1]);
	ofLine(world_far[1], world_far[2]);
	ofLine(world_far[2], world_far[3]);
	ofLine(world_far[3], world_far[0]);
}
