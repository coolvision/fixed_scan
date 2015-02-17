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
    rotation.z = 180.0f;

    // set the camera position
    // in the same way as the arm position is set
    set_arm.resetTransform();
    set_arm.pan(rotation.x);
    set_arm.roll(-rotation.y);
    set_arm.pan(rotation.z);
    set_arm.move(position.x, position.y, position.z);

    // kinect is rotated relative to the arm direction
    // so rotate the camera to make it the same as kinect direction
    set_kinect = set_arm;

//    set_kinect.tilt(kc`A.x);
    set_kinect.pan(-90.0f);
//    set_kinect.roll(kc.z);

    //


    calibrated_kinect_local.resetTransform();
    // in the local coordinate system
//    calibrated_kinect_local.pan(rc.x);
//    calibrated_kinect_local.roll(rc.y);
//    calibrated_kinect_local.pan(rc.z);

    calibrated_kinect_local.tilt(rc.x);
    calibrated_kinect_local.pan(rc.y);
    calibrated_kinect_local.roll(rc.z);

    calibrated_kinect_local.move(pc);

    // transform to the global coordinate system
    ofMatrix4x4 local = calibrated_kinect_local.getGlobalTransformMatrix();
    ofMatrix4x4 global = set_kinect.getGlobalTransformMatrix();
    calibrated_kinect.setTransformMatrix(local * global);

    // draw camera and target position
    ofDrawAxis(500);
    ofDrawGrid(1.0f, 8.0f, false, false, true, false);
    drawVolume();
    ofSetColor(ofColor::white);
    ofDrawSphere(target, 0.005f);
    ofLine(position, target);

    ofPushMatrix();
    ofTranslate(0.0f, floor_y, 0.0f);
    ofDrawGrid(1.0f, 16.0f, false, false, true, false);
    ofPopMatrix();

    // and camera rotation
    set_arm.transformGL();
    ofSetColor(ofColor::red);
    ofDrawCone(0.05, 0.05f);
	ofDrawAxis(0.2);
	set_arm.restoreTransformGL();

//    set_kinect.transformGL();
//    ofSetColor(ofColor::black);
//    ofDrawCone(0.05, 0.05f);
//	ofDrawAxis(0.2);
//	set_kinect.restoreTransformGL();

    calibrated_kinect.transformGL();
    ofSetColor(ofColor::grey);
    ofDrawCone(0.05, 0.05f);
	ofDrawAxis(0.2);
	calibrated_kinect.restoreTransformGL();

    avg_f.vis_step = vis_step;
    avg_f.data.step = data_step;


    if (capture_frames) {
        if (frame_i == 0) {
            //cout << "avg_f.setFromPixels" << endl;
            avg_f.setFromPixels(kinect.getDistancePixels());
            avg_f.setImage(kinect.getPixels());
        } else {
            //cout << "updateWeightedDepth" << endl;
            avg_f.updateWeightedDepth(kinect.getDistancePixels(), weight_a);
            avg_f.updateWeightedImage(kinect.getPixels(), weight_a);
        }

        // make calibration transform matrix
        // CameraOptions calibrated = c_o;
        frame_i++;
    }

    // save the camera position
    avg_f.camera = set_kinect;
    memcpy(c_o.t, set_kinect.getGlobalTransformMatrix().getPtr(),
           16 * sizeof(float));
    avg_f.c = c_o;


    // now, make a "floor map"
    if (add_floor_map) {

        saved_f.push_back(new DepthFrame());
        DepthFrame *s = saved_f.back();
        s->cloneFrom(&avg_f);
        addFloorMap(&avg_f, true);

        add_floor_map = false;
    }


    if (show_avg) {
        CameraOptions calibrated = c_o;
        memcpy(calibrated.t, calibrated_kinect.getGlobalTransformMatrix().getPtr(),
               16 * sizeof(float));
        rangeToWorld(&calibrated, &avg_f, true, max_depth);
        avg_f.meshFromPoints(show_normals);
        avg_f.drawMesh();
    }


    if (kinect.isConnected()) {
        for (int i = 0; i < saved_f.size(); i++) {
            DepthFrame *m = saved_f[i];
            m->c.ref_distance = kinect.getZeroPlaneDistance() + ref_distance;
            m->c.ref_pix_size = kinect.getZeroPlanePixelSize() + ref_pix_size;
        }
    }

    if (draw_plane) {
        drawFloorMaps();
    }
    if (draw_mesh) {
        for (int i = 0; i < saved_f.size(); i++) {
            DepthFrame *m = saved_f[i];
            m->mesh.drawFaces();
        }
    }

    if (clear_floor_maps) {
        clearFloorMaps();
        clear_floor_maps = false;
    }
    if (draw_correspondence) {
        findCorrespondence(true);
    }

    if (re_project) {
        re_project = false;
        reProject(true);
    }

    if (reset_parameters) {
        reset_parameters = false;
        rc.set(0.0f, 0.0f, 0.0f);
        pc.set(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 6; i++) {
            x_curr[i] = FLT_MAX;
            x_next[i] = FLT_MAX;
            f_curr[i] = FLT_MAX;
            f_next[i] = FLT_MAX;
        }
    }

    if (run_gd) {
        gdStep();
        //run_gd = false;
    }
    if (run_plane_gd) {
        planeGDStep();
        //run_plane_gd = false;
    }
    if (plane_correspondence) {
        float f = planeCorrespondence(true);
        plane_correspondence = false;
        cout << "plane " << f << endl;
        findCorrespondence(false);
    }



//==============================================================================
    grab_cam->end();

	ofViewport(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

    ofSetColor(ofColor::gray);
    ofRect(0, 0, SIDEBAR_WIDTH, ofGetWindowHeight());
    gui.draw();

    if (kinect.isConnected()) {

        // get the kinect depth image
        memcpy(depth_data, kinect.getRawDepthPixels(), depth_data_size);

        ConnectedComponents();

        // find which label is in the center
        int center_x = ((img_width/2) / data_step) * data_step;
        int center_y = ((img_height/2) / data_step) * data_step;
        int center_label = labels[center_y * img_width + center_x];

        float w = 200;
        float h = 150;

        // draw the depth data
        int step = data_step * vis_step;
        for (int i = 0; i < img_height; i+=step) {
            for (int j = 0; j < img_width; j+=step) {
                //ofSetColor(depth_data[i * img_width + j] / 10);
                int l = labels[i * img_width + j];
                if (l > 0 && l < N_LABELS) {
                    ofSetColor(label_colors[labels[i * img_width + j]]);
                } else {
                    ofSetColor(ofColor::white);
                }
                if (center_label != 0 && l == center_label) {
                    ofSetColor(ofColor::red);
                }
                ofRect(ofGetWindowWidth() - w + j/step, h + 50 + i/step,
                                1, 1);
            }
        }

        ofSetColor(ofColor::white);
        kinect.drawDepth(ofGetWindowWidth() - w, 50, w, h);

        ofSetColor(ofColor::red);
        ofLine(ofGetWindowWidth() - w, 50 + h/2, ofGetWindowWidth(), 50 + h/2);
        ofLine(ofGetWindowWidth() - w + w/2, 50, ofGetWindowWidth() - w + w/2, 50 + h/2);
    }


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
