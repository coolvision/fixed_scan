/*
 * ofxRangeToWorldCUDA.h
 *
 *  Created on: 2012
 *      Author: sk
 */

#pragma once

#undef USE_CUDA

#include "DepthFrame.h"

// GPU
// under some refactoring, does not work
void rangeToWorldCUDA(CameraOptions *camera, DepthFrame *f);

// CPU
void rangeToWorld(CameraOptions *camera, DepthFrame *f, bool get_normals, float max_depth);
