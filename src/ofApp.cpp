#include "ofApp.h"


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
