//
//  Calibration.cpp
//  fixed_scan
//
//  Created by sk on 9/19/14.
//
//

#include "ofApp.h"

void ofApp::reProject(bool make_mesh) {

    // for all of the saved frames
    for (int i = 0; i < saved_f.size(); i++) {

        // transform to the world coordinates using
        // the calibrated camera transform

        // start with the local coordinate system
        calibrated_kinect_local.resetTransform();
        calibrated_kinect_local.tilt(rc.x);
        calibrated_kinect_local.pan(rc.y);
        calibrated_kinect_local.roll(rc.z);

        calibrated_kinect_local.move(pc);

        ofMatrix4x4 local = calibrated_kinect_local.getGlobalTransformMatrix();

        ofMatrix4x4 global = saved_f[i]->camera.getGlobalTransformMatrix();

        ofCamera set_calibrated;
        set_calibrated.setTransformMatrix(local * global);

        CameraOptions c = saved_f[i]->c;

        memcpy(c.t, set_calibrated.getGlobalTransformMatrix().getPtr(),
               16 * sizeof(float));

        // re-project using calibrated position
        rangeToWorld(&c, saved_f[i], true, max_depth);
        if (make_mesh) {
            saved_f[i]->meshFromPoints(show_normals);
        }

        // and make updated "floor map" out of it
        addFloorMap(saved_f[i], make_mesh);
    }
}

// gradient descent for fitting camera to a horizontal plane
void ofApp::planeGDStep() {

    cout << "planeGDStep " << floor_maps.size() << endl;

    memcpy(&x_curr[0], pc.getPtr(), 3 * sizeof(float));
    memcpy(&x_curr[3], rc.getPtr(), 3 * sizeof(float));

    // current state
    reProject(false);
    float f = planeCorrespondence(false);
    for (int i = 0; i < 6; i++) {
        f_curr[i] = f;
    }

    for (int i = 0; i < 6; i++) {

        if (!opt_parameter[i]) {
            continue;
        }
        memcpy(x_next, x_curr, 6 * sizeof(float));
        x_next[i] = x_curr[i] + gd_epsilon;

        memcpy(pc.getPtr(), &x_next[0], 3 * sizeof(float));
        memcpy(rc.getPtr(), &x_next[3], 3 * sizeof(float));

        reProject(false);
        f_next[i] = planeCorrespondence(false);

        cout << i << " curr " << f_curr[i] << " next " << f_next[i] << endl;
    }

    // now the gradient is:
    for (int i = 0; i < 6; i++) {
        if (!opt_parameter[i]) {
            continue;
        }
        grad[i] = (f_next[i] - f_curr[i]) / gd_epsilon;
        cout << "grad[i] " << i << " " << grad[i] << endl;
    }

    // so the new state is:
    for (int i = 0; i < 6; i++) {
        if (!opt_parameter[i]) {
            continue;
        }
        // need to minimize error
        // if next error is larger, gradient is larger (positive)
        // so need to move away from it
        if (i < 3) {
            x_curr[i] += -grad[i] * (gd_alpha * 0.001);
            cout << "move " << i << " " << -grad[i] * (gd_alpha * 0.001) << endl;
        } else {
            x_curr[i] += -grad[i] * gd_alpha;
            cout << "move " << i << " " << -grad[i] * gd_alpha << endl;
        }

    }
    memcpy(pc.getPtr(), &x_curr[0], 3 * sizeof(float));
    memcpy(rc.getPtr(), &x_curr[3], 3 * sizeof(float));

    float max_pc = 0.3f;
    float min_pc = -0.3f;

    if (pc.x > max_pc) pc.x = max_pc;
    if (pc.x < min_pc) pc.x = min_pc;

    if (pc.y > max_pc) pc.y = max_pc;
    if (pc.y < min_pc) pc.y = min_pc;

    if (pc.z > max_pc) pc.z = max_pc;
    if (pc.z < min_pc) pc.z = min_pc;

    reProject(true);
}

// gradient descent step for calibration parameters fitting
void ofApp::gdStep() {

    memcpy(&x_curr[0], pc.getPtr(), 3 * sizeof(float));
    memcpy(&x_curr[3], rc.getPtr(), 3 * sizeof(float));

    // current state
    reProject(false);
    float f = findCorrespondence(false);
    for (int i = 0; i < 6; i++) {
        f_curr[i] = f;
    }

    for (int i = 0; i < 6; i++) {
        if (!opt_parameter[i]) {
            continue;
        }
        memcpy(x_next, x_curr, 6 * sizeof(float));
        if (i < 3) {
            x_next[i] = x_curr[i] + gd_epsilon * 0.01;
        } else {
            x_next[i] = x_curr[i] + gd_epsilon;
        }


        memcpy(pc.getPtr(), &x_next[0], 3 * sizeof(float));
        memcpy(rc.getPtr(), &x_next[3], 3 * sizeof(float));

        reProject(false);
        f_next[i] = findCorrespondence(false);

        cout << i << " curr " << f_curr[i] << " next " << f_next[i] << endl;
    }

    // now the gradient is:
    for (int i = 0; i < 6; i++) {
        if (!opt_parameter[i]) {
            continue;
        }
        grad[i] = (f_next[i] - f_curr[i]) / gd_epsilon;
    }

    // so the new state is:
    for (int i = 0; i < 6; i++) {
        if (!opt_parameter[i]) {
            continue;
        }
        // need to minimize error
        // if next error is larger, gradient is larger (positive)
        // so need to move away from it
        if (i < 3) {
            x_curr[i] += -grad[i] * (gd_alpha * 0.01);
            cout << "m " << i << " " << -grad[i] * (gd_alpha * 0.001) << endl;
        } else {
            x_curr[i] += -grad[i] * gd_alpha;
            cout << "m " << i << " " << -grad[i] * gd_alpha << endl;
        }
    }
    memcpy(pc.getPtr(), &x_curr[0], 3 * sizeof(float));
    memcpy(rc.getPtr(), &x_curr[3], 3 * sizeof(float));
    
    reProject(true);
}