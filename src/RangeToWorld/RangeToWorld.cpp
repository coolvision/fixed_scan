//
//  ofxRangeToWorld.cpp
//  kinectExample
//
//  Created by sk on 8/25/14.
//
//

#include <stdio.h>

#include "ofxRangeToWorld.h"

// from the image to the world
ofPoint cameraBackProject(CameraOptions *camera, const ofPoint &v) {

	ofPoint result;
	float factor = 2 * camera->ref_pix_size * v.z / camera->ref_distance;
	result.x = (v.x - camera->x_resolution / 2) * factor;
	result.y = (v.y - camera->y_resolution / 2) * factor;
	result.z = v.z;

	// camera parameters are set for mm coords, transform to meters
	result /= 1000.0f;

	return result;
}

ofPoint matrixVectorMultiply(const float* t, const ofPoint &v) {
	ofPoint result;
	float d = 1.0f / (t[3] * v.x + t[7] * v.y + t[11] * v.z + t[15]);
	result.x = (t[0]*v.x + t[4]*v.y + t[8]*v.z + t[12])*d;
	result.y = (t[1]*v.x + t[5]*v.y + t[9]*v.z + t[13])*d;
	result.z = (t[2]*v.x + t[6]*v.y + t[10]*v.z + t[14])*d;
	return result;
}

ofPoint getWorldCoordinate(CameraOptions *camera, const ofPoint &v) {
	ofPoint w, result;

	w = cameraBackProject(camera, v);
	result = matrixVectorMultiply(camera->t, w);
    
	return result;
}

void rangeToWorld(CameraOptions *camera, DepthFrame *f, bool get_normals, float max_depth) {

    int step = f->data.step;

    // for all pixels, transform into world coordinates
    for (int y = 0; y < f->data.height; y += step) {
        for (int x = 0; x < f->data.width; x += step) {

            // flip (for some reason, a quck hack...)
            int set_x = x;

            uint i = y * f->data.width + x;
            uint p_i = i * 3;

            // change depth direction, so it would correspond to openGL coords
            ofPoint d(set_x, y, f->data.depth[i]);

            bool valid_d = false;

            if (d.z < max_depth && d.z > 5) {
                valid_d = true;
            }

            ofPoint p = getWorldCoordinate(camera, d);

            // 3d space filtering
            if (valid_d && p.x > camera->min[0] && p.x < camera->max[0] &&
                p.y > camera->min[1] && p.y < camera->max[1] &&
                p.z > camera->min[2] && p.z < camera->max[2]) {

                f->data.points[p_i] = p.x;
                f->data.points[p_i + 1] = p.y;
                f->data.points[p_i + 2] = p.z;
                
            } else {
                
                f->data.points[p_i] = -FLT_MAX;
                f->data.points[p_i + 1] = -FLT_MAX;
                f->data.points[p_i + 2] = -FLT_MAX;
            }
        }
    }

    if (get_normals) {
        for (int y = 0; y < f->data.height-step; y += step) {
            for (int x = 0; x < f->data.width-step; x += step) {

                // flip (for some reason, a quck hack...)
                //int set_x = f->data.width - x;
                int set_x = x;

                uint i = y * f->data.width + x;
                uint p_i = i * 3;

                // estimate a normal
                ofPoint p1, p2, p3;
                getPoint(&f->data, &p1, set_x + step, y);
                getPoint(&f->data, &p2, set_x, y + step);
                getPoint(&f->data, &p3, set_x, y);

                ofPoint normal;

                if (p1.z == 0.0 || p2.z == 0 || p3.z == 0) {

                    normal.x = 0.0f;
                    normal.y = 0.0f;
                    normal.z = 0.0f;

                } else {
                    
                    ofPoint v1 = p1 - p3;
                    ofPoint v2 = p2 - p3;
                    
                    normal = v1.cross(v2);
                    normal = normal.normalized();
                }
                
                f->data.normals[p_i] = normal.x;
                f->data.normals[p_i + 1] = normal.y;
                f->data.normals[p_i + 2] = normal.z;
            }
        }
    }
}