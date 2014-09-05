#include "ofApp.h"

void ofApp::clearFloorMaps() {

    for (int i = 0; i < floor_maps.size(); i++) {
        floor_maps[i]->releaseHost();
        delete floor_maps[i];
    }
    floor_maps.clear();

    for (int i = 0; i < saved_f.size(); i++) {
        saved_f[i]->releaseHost();
        delete saved_f[i];
    }
    saved_f.clear();
}

void ofApp::drawCorrespondence() {

    if (floor_maps.empty()) {
        return;
    }
    DepthFrame *m = floor_maps.back();

    ofPoint offset = target - ofPoint(0.5f, 0.5f, 0.5f);
    ofPoint p;
    ofPoint p2;
    for (int y = 0; y < m->data.height; y++) {
		for (int x = 0; x < m->data.width; x++) {

            bool found_undefined = false;

            for (int i = 0; i < floor_maps.size(); i++) {
                FrameData *d = &floor_maps[i]->data;

                p.y = d->depth[y * d->width + x];

                if (p.y == -FLT_MAX) {
                    found_undefined = true;
                    break;
                }
                if (found_undefined) {
                    break;
                }
            }

            if (found_undefined) {
                continue;
            }

            // all values are present
            p.x = x * 0.005 + offset.x;
            p.z = y * 0.005 + offset.z;

            p2 = p;

            ofSetColor(ofColor::white);
            for (int i = 0; i < floor_maps.size()-1; i++) {
                FrameData *d = &floor_maps[i]->data;
                p2.y = d->depth[y * d->width + x];
                ofLine(p, p2);
            }


        }
    }
}

void ofApp::addFloorMap(DepthFrame *f) {

    saved_f.push_back(new DepthFrame());
    DepthFrame *s = saved_f.back();
    s->cloneFrom(f);

    floor_maps.push_back(new DepthFrame());
    DepthFrame *m = floor_maps.back();
    int step = f->data.step;

    m->init(f->data.width, f->data.height);
    m->allocateHost();
    m->data.step = f->data.step;
    m->vis_step = f->vis_step;

    // reset
	for (int y = 0; y < f->data.height; y++) {
		for (int x = 0; x < f->data.width; x++) {
            // undefined value
            m->data.depth[y * f->data.width + x] = -FLT_MAX;
        }
    }

    // look at all of the poitns for the input data
    // put them into the depth map depending on the x, z coordinates
    ofPoint p;
    ofPoint offset = target - ofPoint(0.5f, 0.5f, 0.5f);
	for (int y = 0; y < f->data.height; y += step) {
		for (int x = 0; x < f->data.width; x += step) {

            getPoint(&f->data, &p, x, y);

            ofPoint c = p;
            c -= offset;
            c /= 0.005f;
            int j = (int)c.x;
            int i = (int)c.z;

            if (i >= 0 && i < m->data.height && j >= 0 && j < m->data.width) {
                m->data.depth[i * f->data.width + j] = p.y;
            }
        }
    }

    // now make a mesh out of the set points

    ofMesh &mesh = m->mesh;
	ofPoint local_points[2][2];
	mesh.clear();


    ofColor color;
    int map_i = 0;
    if (!floor_maps.empty()) {
        map_i = floor_maps.size() - 1;
    }
    if (map_i < colors.size()) {
        color = colors[map_i];
    } else {
        color = ofColor::white;
    }
    ofPoint normal(0.0f, 1.0f, 0.0f);

	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    step = 1;
    for (int i = 0; i < f->data.height-step; i += step) {
		for (int j = 0; j < f->data.width-step; j += step) {

            float d = m->data.depth[i * f->data.width + j];
            if (d == -FLT_MAX) {
                continue;
            }

			for (int k  = 0; k <= 1; k++) {
				for (int l = 0; l <= 1; l++) {
                    p.x = (j+l*step) * 0.005 + offset.x;
                    p.z = (i+k*step) * 0.005 + offset.z;
                    p.y = m->data.depth[(i+k*step) * f->data.width + (j+l*step)];

                    if (p.y == -FLT_MAX) {
                        p.y = d;
                    }
                    p.y = d;

                    local_points[k][l] = p;
                }
			}

            mesh.addVertex(local_points[0][0]);
			mesh.addColor(color);
			mesh.addNormal(normal);
			mesh.addVertex(local_points[1][0]);
			mesh.addColor(color);
			mesh.addNormal(normal);
			mesh.addVertex(local_points[0][1]);
			mesh.addColor(color);
			mesh.addNormal(normal);

			mesh.addVertex(local_points[1][1]);
			mesh.addColor(color);
			mesh.addNormal(normal);
			mesh.addVertex(local_points[0][1]);
			mesh.addColor(color);
			mesh.addNormal(normal);
			mesh.addVertex(local_points[1][0]);
			mesh.addColor(color);
			mesh.addNormal(normal);
        }
    }
}

void ofApp::drawFloorMaps() {
    for (int i = 0; i < floor_maps.size(); i++) {
        DepthFrame *m = floor_maps[i];
        m->mesh.drawWireframe();
    }

    for (int i = 0; i < saved_f.size(); i++) {
        DepthFrame *m = saved_f[i];
        m->mesh.drawFaces();
    }
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

//		case'p':
//			bDrawPointCloud = !bDrawPointCloud;
//			break;

//		case 'w':
//			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
//			break;

//		case 'o':
//			kinect.setCameraTiltAngle(angle); // go back to prev tilt
//			kinect.open();
//			break;

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
