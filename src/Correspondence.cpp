//
//  Correspondence.cpp
//  fixed_scan
//
//  Created by sk on 9/9/14.
//
//

#include "ofApp.h"

float ofApp::findCorrespondence(bool draw) {

    ofPushStyle();

    if (floor_maps.empty()) {
        return;
    }
    DepthFrame *m = floor_maps.back();

    float sum = 0.0f;
    int n_corresponding = 0;

    ofPoint offset = target - ofPoint(1.0f, 1.0f, 1.0f);
    float map_step = 0.005f;

    ofPoint p;
    ofPoint p2;
    for (int y = 0; y < m->data.height; y++) {
		for (int x = 0; x < m->data.width; x++) {

            bool undefined = false;

            for (int i = 0; i < floor_maps.size(); i++) {
                FrameData *d = &floor_maps[i]->data;
                p.y = d->depth[y * d->width + x];
                if (p.y == -FLT_MAX) {
                    undefined = true;
                    break;
                }
            }
            if (undefined) {
                continue;
            }

            // all values are present
            p.x = x * map_step + offset.x;
            p.z = y * map_step + offset.z;
            p2 = p;

            if (draw) {
                ofSetColor(ofColor::white);
            }
            for (int i = 0; i < floor_maps.size(); i++) {
                FrameData *d = &floor_maps[i]->data;
                p2.y = d->depth[y * d->width + x];

                if (p2.y == -FLT_MAX) {
                    continue;
                }

                if (draw) {
                    ofLine(p, p2);
                }
                sum += abs(p2.y - p.y);
                n_corresponding++;
            }
        }
    }

    if (n_corresponding > 0) {
        sum /= (float)n_corresponding;
    }

    if (!draw) {
        cout << "c: " << sum * 1000.0f << " n " << n_corresponding << endl;
    }
    
    //return sum * 1000.0f;
    return 0.3 * (sum * 1000.0f) - n_corresponding / 1000.0f;

    ofPopStyle();
}

float ofApp::planeCorrespondence(bool draw) {

    if (floor_maps.empty()) {
        return;
    }
    DepthFrame *m = floor_maps.back();

    float sum = 0.0f;
    int n_corresponding = 0;

    ofPoint offset = target - ofPoint(1.0f, 1.0f, 1.0f);
    float map_step = 0.005f;

    ofPoint p;
    ofPoint p2;

    // find the average y
    float avg_y = 0.0f;
    int n = 0;
    for (int y = 0; y < m->data.height; y++) {
		for (int x = 0; x < m->data.width; x++) {

            for (int i = 0; i < floor_maps.size(); i++) {
                FrameData *d = &floor_maps[i]->data;

                p.y = d->depth[y * d->width + x];

                if (p.y != -FLT_MAX) {
                    avg_y += p.y;
                    n++;
                }
            }
        }
    }

    if (n > 0) {
        avg_y /= (float)n;
    }
    floor_y = avg_y;

    cout << "floor_y " << floor_y << endl;

    // now find the deviation
    float deviation = 0.0f;
    n = 0;
    for (int y = 0; y < m->data.height; y++) {
		for (int x = 0; x < m->data.width; x++) {

            for (int i = 0; i < floor_maps.size(); i++) {
                FrameData *d = &floor_maps[i]->data;

                p.y = d->depth[y * d->width + x];

                if (p.y != -FLT_MAX) {
                    deviation += abs(p.y - avg_y);
                    n++;
                }
            }
        }
    }
    if (n > 0) {
        deviation /= (float)n;
    }

    cout << "d: " << deviation * 1000.0f << " n " << n << endl;

    return deviation * 1000.0f;
}


