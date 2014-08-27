//
//  Update.cpp
//  kinectExample
//
//  Created by sk on 8/23/14.
//
//

#include "ofApp.h"


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