//
//  FloorMaps.cpp
//  fixed_scan
//
//  Created by sk on 9/19/14.
//
//

#include "ofApp.h"

void ofApp::addFloorMap(DepthFrame *f, bool make_mesh) {

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

    // look at all of the points for the input data
    // put them into the depth map depending on the x, z coordinates
    ofPoint p;
    ofPoint offset = target - ofPoint(1.0f, 1.0f, 1.0f);
    float map_step = 0.005f;
	for (int y = 0; y < f->data.height; y += step) {
		for (int x = 0; x < f->data.width; x += step) {

            getPoint(&f->data, &p, x, y);

            ofPoint c = p;
            c -= offset;
            c /= map_step;
            int j = (int)c.x;
            int i = (int)c.z;

            if (i >= 0 && i < m->data.height && j >= 0 && j < m->data.width) {

                float d = m->data.depth[i * f->data.width + j];
                if (d != -FLT_MAX) {
                    if (p.y > d) {
                        m->data.depth[i * f->data.width + j] = p.y;
                    }
                } else {
                    m->data.depth[i * f->data.width + j] = p.y;
                }
            }
        }
    }

    // now make a mesh out of the set points
    if (make_mesh) {
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
                        p.x = (j+l*step) * map_step + offset.x;
                        p.z = (i+k*step) * map_step + offset.z;
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
}

void ofApp::loadMaps() {

    clearFloorMaps();

    ofDirectory data_dir;
    data_dir.listDir("load");
    data_dir.sort();
    cout << "data_dir.size() " << data_dir.size() << endl;
    cout << data_dir.getAbsolutePath() << endl;

    ofDirectory dir;

    for (int i = 0; i < (int) data_dir.size(); i++) {

        dir.open(data_dir.getPath(i));

        if (!dir.isDirectory()) {
            cout << "load " << dir.getAbsolutePath() << endl;

            ifstream infile;
            infile.open(dir.getAbsolutePath().c_str());

            saved_f.push_back(new DepthFrame());
            DepthFrame *f = saved_f.back();
            int step = 1;

            f->init(640, 480);
            f->allocateHost();

            infile >> step;
            f->data.step = step;
            f->vis_step = vis_step;

            f->c = c_o;
            infile >> f->c.ref_pix_size;
            infile >> f->c.ref_distance;
            infile >> f->c.x_resolution;
            infile >> f->c.y_resolution;

            cout << "step " << step << endl;
            for (int j = 0; j < 16; j++) {
                infile >> f->c.t[j];
                cout << f->c.t[j] << endl;
            }
            ofMatrix4x4 matrix;
            matrix.set(f->c.t);
            f->camera.setTransformMatrix(matrix);

            for (int y = 0; y < f->data.height; y += step) {
                for (int x = 0; x < f->data.width; x += step) {
                    int p_i = y * f->data.width + x;
                    infile >> f->data.depth[p_i];
                }
            }

            for (int y = 0; y < f->data.height; y += step) {
                for (int x = 0; x < f->data.width; x += step) {
                    int p_i = y * f->data.width + x;
                    p_i *= 3;
                    char ch[3];

                    infile.read(ch, 3);

                    f->data.image[p_i] = ch[0];
                    f->data.image[p_i+1] = ch[1];
                    f->data.image[p_i+2] = ch[2];
                }
            }
            infile.close();


            cout << "step0 " << f->data.step << endl;

            rangeToWorld(&f->c, f, true, max_depth);
            f->meshFromPoints(show_normals);
            addFloorMap(f, true);
        }
    }
}

void ofApp::saveMaps() {

    string time = ofGetTimestampString();

    // need to save only the points cloud data (and images)
    for (int i = 0; i < saved_f.size(); i++) {

        DepthFrame *f = saved_f[i];

        ofDirectory dir("maps");

        string name = dir.getAbsolutePath() + "/" + time + "_data" + ofToString(i) + ".txt";
        ofstream outfile;
        outfile.open(name.c_str());

        cout << "save " << name << endl;
        int step = f->data.step;

        outfile << step << endl;
        outfile << saved_f[i]->c.ref_pix_size << endl;
        outfile << saved_f[i]->c.ref_distance << endl;
        outfile << saved_f[i]->c.x_resolution << endl;
        outfile << saved_f[i]->c.y_resolution << endl;
        float *m = saved_f[i]->camera.getGlobalTransformMatrix().getPtr();
        for (int j = 0; j < 16; j++) {
            outfile << m[j] << endl;
        }

        float *d = f->data.depth;
        uchar *img = f->data.image;
        for (int y = 0; y < f->data.height; y += step) {
            for (int x = 0; x < f->data.width; x += step) {
                int p_i = y * f->data.width + x;
                outfile << f->data.depth[p_i] << " ";
            }
            outfile << endl;
        }

        for (int y = 0; y < f->data.height; y += step) {
            for (int x = 0; x < f->data.width; x += step) {
                int p_i = y * f->data.width + x;
                p_i *= 3;
                outfile << f->data.image[p_i];
                outfile << f->data.image[p_i+1];
                outfile << f->data.image[p_i+2];
            }
        }
        outfile.close();
    }
}

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

void ofApp::drawFloorMaps() {
    for (int i = 0; i < floor_maps.size(); i++) {
        DepthFrame *m = floor_maps[i];
        m->mesh.drawWireframe();
    }
}

