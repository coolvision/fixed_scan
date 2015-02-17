/*
 * ConnectedComponents.cpp
 *
 *  Created on: Sep 25, 2013
 *      Author: sk
 */

#include "ofApp.h"

void init_lables(int *Parent, int n) {

    for (int i = 0; i < n; i++) {
        Parent[i] = 0;
    }
}

int find_parent_label(int x, int *Parent) {

    int j = x;
    while (Parent[j] != 0) {
        j = Parent[j];
    }
    return j;
}

void union_labels(int x, int y, int *Parent) {

    int j = find_parent_label(x, Parent);
    int k = find_parent_label(y, Parent);

    if (j != k) {
        Parent[k] = j;
    }
}

void ofApp::ConnectedComponents() {

    int parent[N_LABELS];
    int inc_label = 1;
    int curr_label = 1;
    int n_filled = 0;
    init_lables(parent, N_LABELS);

    int n4[2];

    for (int i = 0; i < img_height; i++) {
        for (int j = 0; j < img_width; j++) {
            labels[i * img_width + j] = 0;
        }
    }

    // look at all of the cells in the bounding box
    int step = data_step;
    for (int i = 0; i < img_height; i+=step) {
        for (int j = 0; j < img_width; j+=step) {

            if (i-step < 0 || j-step < 0) {
                continue;
            }

            int d = depth_data[i * img_width + j];

            if (d > max_depth || d < 100) {
                continue;
            }

            n_filled = 0;
            n4[0] = -1;
            n4[1] = -1;

            int d1 = depth_data[i * img_width + (j-step)];
            if (d1 < max_depth && d1 > 100) {
                n_filled++;
                n4[0] = labels[i * img_width + (j-step)];
            }
            int d2 = depth_data[(i-step) * img_width + j];
            if (d2 < max_depth && d2 > 100) {
                n_filled++;
                n4[1] = labels[(i-step) * img_width + j];
            }

            if (n_filled == 0) {
                // assign new label
                if (inc_label < N_LABELS - 1) {
                    inc_label++;
                } else {
                    cout << "inc_label > N_LABELS - 1" << endl;
                }
                curr_label = inc_label;
            } else {
                // find min. value label
                int min_l = N_LABELS + 1;
                for (int k = 0; k < 2; k++) {
                    if (n4[k] > 0 && n4[k] < min_l) {
                        min_l = n4[k];
                    }
                }
                if (min_l == N_LABELS + 1) {
                    curr_label = 0;
                } else {
                    curr_label = min_l;
                }
            }

            labels[i * img_width + j] = curr_label;

            if (n_filled > 0) {
                for (int k = 0; k < 2; k++) {
                    if (n4[k] >= 0 && n4[k] != curr_label) {
                        union_labels(curr_label, n4[k], parent);
                    }
                }
            }
        }
    }

    // now set the final labels to all cells
    for (int i = 0; i < img_height; i+=step) {
        for (int j = 0; j < img_width; j+=step) {
            labels[i * img_width + j] =
                find_parent_label(labels[i * img_width + j], parent);
        }
    }
}

