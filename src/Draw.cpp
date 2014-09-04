//
//  Draw.cpp
//  kinectExample
//
//  Created by sk on 8/23/14.
//
//

#include "ofApp.h"

void ofApp::draw() {

	ofSetColor(255, 255, 255);

    ofViewport(SIDEBAR_WIDTH, 0, ofGetWindowWidth() - SIDEBAR_WIDTH,
               ofGetWindowHeight());
    grab_cam->begin();
//==============================================================================


    // compute absolute position and rotation
    position.y = sp.x;
    position.x = sp.y;
    position.z = 0;
    position.rotate(sp.z, ofPoint(0.0f, 1.0f, 0.0f));
    // setting relative to the target
    position += target;

    // set camera to point to the target
    rotation.x = sp.z;
    //rotation.y = 90 + atan(sp.y / sp.x) * RAD_TO_DEG;
    rotation.y = camera_tilt;
    rotation.z = 0.0f;


    // set the camera position
    // in the same way as the arm position is set
    camera.resetTransform();
    camera.pan(rotation.x);
    camera.roll(-rotation.y);
    camera.pan(rotation.z);
    camera.move(position.x, position.y, position.z);


    // draw camera and target position
    ofDrawAxis(500);
    ofDrawGrid(1.0f, 8.0f, false, false, true, false);
    drawVolume();
    ofSetColor(ofColor::white);
    ofDrawSphere(target, 0.005f);
    ofLine(position, target);
    // and camera rotation
    camera.transformGL();
    ofSetColor(ofColor::black);
    ofDrawBox(0.1);
	ofDrawAxis(0.2);
	camera.restoreTransformGL();

    // kinect is rotated relative to the arm direction
    // so rotate the camera to make it the same as kinect direction
    set_kinect = camera;
    set_kinect.pan(90.0f);

//    curr_f.setFromPixels(kinect.getDistancePixels());
//    curr_f.setImage(kinect.getPixels());
//    curr_f.vis_step = vis_step;
//    curr_f.data.step = data_step;

    avg_f.vis_step = vis_step;
    avg_f.data.step = data_step;

    // fill in the camera option
    if (kinect.isConnected()) {
        memcpy(c_o.t, set_kinect.getGlobalTransformMatrix().getPtr(),
               16 * sizeof(float));
    }

    //rangeToWorld(&c_o, &curr_f, true);
    //curr_f.meshFromPoints(true);
    //curr_f.drawMesh();


    if (capture_frames) {
        if (frame_i == 0) {
            cout << "avg_f.setFromPixels" << endl;
            avg_f.setFromPixels(kinect.getDistancePixels());
            avg_f.setImage(kinect.getPixels());
        } else {
            cout << "updateWeightedDepth" << endl;
            avg_f.updateWeightedDepth(kinect.getDistancePixels(), weight_a);
            avg_f.updateWeightedImage(kinect.getPixels(), weight_a);
        }
        rangeToWorld(&c_o, &avg_f, true);
        avg_f.meshFromPoints(show_normals);

        frame_i++;
    }
    avg_f.drawMesh();


    // now, make a "floor map"
    if (add_floor_map) {
        addFloorMap(&avg_f);
        add_floor_map = false;
    }
    drawFloorMaps();



//==============================================================================
    grab_cam->end();

	ofViewport(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

    if(!bDrawPointCloud) {
        kinect.drawDepth(SIDEBAR_WIDTH, 100, 400, 300);
    }
    ofSetColor(ofColor::gray);
    ofRect(0, 0, SIDEBAR_WIDTH, ofGetWindowHeight());
    gui.draw();
}

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
