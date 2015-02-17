/*
 * DepthFrame.cpp
 *
 *  Created on: 2012
 *      Author: sk
 */

#include "DepthFrame.h"

void getPoint(FrameData *f, ofPoint *point, int x, int y) {

	int index = (y * f->width + x) * 3;
	point->x = f->points[index];
	point->y = f->points[index + 1];
	point->z = f->points[index + 2];
}

void getNormal(FrameData *f, ofPoint *point, int x, int y) {

	int index = (y * f->width + x) * 3;
	point->x = f->normals[index];
	point->y = f->normals[index + 1];
	point->z = f->normals[index + 2];
}

void DepthFrame::cloneFrom(DepthFrame *f) {

    (*this) = (*f);
    init(f->data.width, f->data.height);
    allocateHost();

    memcpy(data.depth, f->data.depth, data.depth_size);
    memcpy(data.points, f->data.points, data.points_size);
    memcpy(data.normals, f->data.normals, data.points_size);
    memcpy(data.image, f->data.image, data.image_size);

    data.step = f->data.step;
}

void DepthFrame::init(int width, int height) {

	data.depth = NULL;
	data.points = NULL;
	data.normals = NULL;
	data.image = NULL;

	data.width = width;
	data.height = height;
	data.step = 1;

	data.depth_size = width * height * sizeof(float);
	data.points_size = width * height * 3 * sizeof(float);
    data.image_size = width * height * 3 * sizeof(uint8_t);
}

void DepthFrame::updateWeightedDepth(float *pixels, float a) {

	int step = data.step;

    for (int y = 0; y < data.height; y += step) {
        for (int x = 0; x < data.width; x += step) {

            uint i = y * data.width + x;

            float v = data.depth[i];
            float u = pixels[i];

            if (u > 0.001f) {
                if (v < 0.001f) {
                    data.depth[i] = u;
                } else {
                    data.depth[i] = v * a + u * (1.0f - a);
                }
            }
        }
    }
}

void DepthFrame::updateWeightedImage(uint8_t *img, float a) {

	int step = data.step;

    uint8_t *p, *q;
    for (int y = 0; y < data.height; y += step) {
        for (int x = 0; x < data.width; x += step) {

            uint i = y * data.width + x;

            p = &data.image[i];
            q = &img[i];

            for (int j = 0; j < 3; j++) {
                float v = *p++;
                float u = *q++;

                data.image[i+j] = v * a + u * (1.0f - a);
            }
        }
    }
}

void DepthFrame::setFromPixels(float *pixels) {

	memcpy(data.depth, pixels, data.depth_size);
}

void DepthFrame::setImage(uint8_t *img) {

	memcpy(data.image, img, data.image_size);
}

void DepthFrame::allocateDevice() {
#ifdef USE_CUDA
	dev.depth_bn = depth_bn;
	dev.points_bn = points_bn;

	cudaMalloc((void **) &dev.depth, depth_bn);
	cudaMalloc((void **) &dev.points, points_bn);
	cudaMalloc((void **) &dev.normals, points_bn);
#endif
}

void DepthFrame::releaseDevice() {
#ifdef USE_CUDA
	cudaFree(dev.depth);
	cudaFree(dev.points);
	cudaFree(dev.normals);
#endif
}

void DepthFrame::allocateHost() {

	data.depth = (float *) malloc(data.depth_size);
	data.points = (float *) malloc(data.points_size);
	data.normals = (float *) malloc(data.points_size);
    data.image = (uint8_t *) malloc(data.image_size);
}

void DepthFrame::releaseHost() {

	free(data.depth);
	free(data.points);
	free(data.normals);
	free(data.image);
}

void DepthFrame::meshFromPoints(bool draw_normals) {

	// make a mesh from returned data
	ofPoint p;
	ofPoint normal;
	ofPoint local_points[2][2];

	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);

    int step = data.step * vis_step;

    //cout << "meshFromPoints " << data.step << " " << vis_step << " " << step << endl;

	for (int y = 0; y < data.height-step; y += step) {
		for (int x = 0; x < data.width-step; x += step) {

			bool zero_depth_found = false;
            getPoint(&data, &p, x, y);
			for (int i = 0; i <= 1; i++) {
				for (int j = 0; j <= 1; j++) {
					getPoint(&data, &local_points[i][j],
							x + i * step, y + j * step);
					if (local_points[i][j].z == -FLT_MAX) {
						zero_depth_found = true;
						break;
					}
                    ofPoint v = (local_points[i][j] - p);
                    float d = v.length();
                    if (d > 0.05f) {
						zero_depth_found = true;
						break;
                    }
				}
                if (zero_depth_found) {
                    break;
                }
			}

			if (zero_depth_found) {
				continue;
            }

			ofColor color;
			if (draw_normals) {
                getNormal(&data, &normal, x, y);
				color.set(abs(normal.x) * 255.0,
						abs(normal.y) * 255.0,
						abs(normal.z) * 255.0, 200.0f);
			} else {
                int i = y * data.width + x;
                int p_i = i * 3;
                color.set(data.image[p_i],
                          data.image[p_i+1], data.image[p_i+2]);
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

void DepthFrame::drawMesh() {

	// draw the mesh
	mesh.drawFaces();
}
