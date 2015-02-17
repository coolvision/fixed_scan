/*
 * FrameData.h
 *
 *  Created on: 2012
 *      Author: sk
 */

#pragma once

class FrameData {
public:

    short *raw_depth;

	float *depth;
	float *points;
	float *normals;
	uint8_t *image; // RGB image

	size_t depth_size;
	size_t points_size;
    size_t image_size;

	int width;
	int height;
	int step;
};

void getPoint(FrameData *f, ofPoint *point, int x, int y);
void getNormal(FrameData *f, ofPoint *point, int x, int y);

class CameraOptions {
public:
	float t[16]; // transform matrix
	float it[16]; // inverse transform
	float ref_pix_size;
	float ref_distance;
    int x_resolution;
    int y_resolution;

	// 3d volume filtering
	float min[3];
	float max[3];
};

