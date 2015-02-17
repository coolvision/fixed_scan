/*
 * DepthFrame.h
 *
 *  Created on: 2012
 *      Author: sk
 */

#pragma once

#include "ofMain.h"
#include "FrameData.h"

class DepthFrame {
public:

    ofCamera camera;
    CameraOptions c;

	// data on the CPU
	FrameData data;

    // data on the GPU
	//FrameDataCUDA dev;

	// visualization
    int vis_step;
	ofMesh mesh;
	ofImage depth_image;
	ofImage color_image;

    void cloneFrom(DepthFrame *f);

	void init(int width, int height);
	void setFromPixels(float *pixels);
    void setImage(uint8_t *img);
    void updateWeightedDepth(float *pixels, float a);
    void updateWeightedImage(uint8_t *img, float a);

	void allocateHost();
	void releaseHost();

	void allocateDevice();
	void releaseDevice();

	void meshFromPoints(bool draw_normals);
	void drawMesh();
	void drawNormals();
};
