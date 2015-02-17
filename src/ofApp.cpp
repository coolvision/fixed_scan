#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {



	}

#ifdef LEAP_DEMO
    vector <Hand> hands = leap.getLeapHands();
    if( leap.isFrameNew() && hands.size() ){
        if (hands.size() == 1) {
            Hand &h = hands[0];

            if (h.grabStrength() > 0.9) {
                //cout << "grab " << frames_grabbed_n << " " << h.grabStrength() << endl;

                if (prev_grab_state == false) {
                    start_p.set(h.palmPosition().x, h.palmPosition().y,
                                h.palmPosition().z);
                }

                frames_grabbed_n++;
                prev_grab_state = true;
            } else {
                //cout << "no grab " << h.grabStrength() << endl;

                if (prev_grab_state == true && frames_grabbed_n > 10) {
                    ofPoint end_p(h.palmPosition().x, h.palmPosition().y,
                                  h.palmPosition().z);
                    ofPoint move_v = end_p - start_p;

                    cout << "MOVE!" << move_v.x << " " << move_v.y
                    << " " << move_v.z << endl;

                    curr_p += move_v;
                    ofLoadURL(url +
                              "x=" + ofToString(curr_p.z) + "&" +
                              "y=" + ofToString(curr_p.x) + "&" +
                              "z=" + ofToString(curr_p.y) + "&" +
                              "yaw=" + ofToString(0) + "&" +
                              "pitch=" + ofToString(90) + "&" +
                              "roll=" + ofToString(180));
                }

                frames_grabbed_n = 0;
                prev_grab_state = false;
            }

        }
    }
#endif
}


//--------------------------------------------------------------
void ofApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
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

            //
            // reset position to default
            ofLoadURL(url +
                      "x=" + ofToString(0) + "&" +
                      "y=" + ofToString(0) + "&" +
                      "z=" + ofToString(150) + "&" +
                      "yaw=" + ofToString(0) + "&" +
                      "pitch=" + ofToString(90) + "&" +
                      "roll=" + ofToString(180));

            // TRANS(500,0,150,yaw,pitch,roll)

            // for Leap Motion demo
            // curr_p.set(0,0,150);

            break;

		case'f':
            //			save_points = true;

            capture_frames = !capture_frames;

            frame_i = 0;

            // do the next snapshot
            if (!capture_frames) {
                save_i++;
            }
			break;

            //		case'n':
            //            pc.set(0.0f, 0.0f, 0.0f);
            //            rc.set(0.0f, 0.0f, 0.0f);
            //			break;

		case'c':

            // clear all saved data...
            save_i = 0;
			//meshes.clear();
            //camera_positions.clear();
			break;

		case'p':
			//bDrawPointCloud = !bDrawPointCloud;

            re_project = true;

			break;

		case 'a':
			add_floor_map = true;
			break;

		case 'q':
            saveMaps();
			break;

		case 'l':
            loadMaps();
			break;

            //		case 'c':
            //			kinect.setCameraTiltAngle(0); // zero the tilt
            //			kinect.close();
            //			break;

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
