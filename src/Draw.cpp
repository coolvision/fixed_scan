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

    ofDrawAxis(500);
    ofDrawGrid(1.0f, 8.0f, false, false, true, false);
    drawVolume();

    // compute absolute position and rotation
    position.y = sp.x;
    position.x = sp.y;
    position.z = 0;
    position.rotate(sp.z, ofPoint(0.0f, 1.0f, 0.0f));
    position += target;

    rotation.x = sp.z;  // MAY BE INCORRECT
    rotation.y = 90 + atan(sp.y / sp.x) * RAD_TO_DEG;
    rotation.z = 0.0f;


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

    drawPointCloud();


    camera.roll(-90.0f); // MAY BE INCORRECT
    camera.tilt(-90.0f);
    if (save_points) {
        savePoints();
        save_points = false;
    }


    camera.transformGL();
    drawCurrPointCloud();
	camera.restoreTransformGL();


    grab_cam->end();

	ofViewport(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

    if(!bDrawPointCloud) {
        kinect.drawDepth(SIDEBAR_WIDTH, 100, 400, 300);
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
