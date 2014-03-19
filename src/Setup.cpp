/*
 * Setup.cpp
 *
 *  Created on: 2012
 *      Author: sk
 */

#include "ShapeApp.h"

#include <cuda_runtime.h>
#include <helper_cuda.h>    // includes cuda.h and cuda_runtime_api.h
int divUp(int a, int b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

void ShapeApp::setupUI() {

	grab_cam = new ofxGrabCam(false);
	grab_cam->reset();
	grab_cam->setPosition(1.0, 1.0, 1.0);
	grab_cam->lookAt(ofVec3f(0.0f, 0.0f, 0.0f));

	gui.setDefaultKeys(true);
	gui.setDraw(true);

	gui.addFPSCounter();

	gui.addSlider("image_scale", ui_image_scale, 0.2, 3.0);
	gui.addToggle("depth", ui_show_depth);

	gui.addTitle("voxels");
	gui.addToggle("update_voxels", ui_update_voxels);
	gui.addToggle("estimate_maps", ui_estimate_maps);
	gui.addButton("reset", ui_reset_voxels);
	gui.addButton("reset_weights", ui_reset_voxel_weights);

	gui.addTitle("rays");
	gui.addSlider("march_step", ui_march_step, 0.0005, 0.1);
	gui.addSlider("iterations_n", ui_march_iterations_n, 10, 1000);

	gui.loadFromXML();
	gui.setAlignRight(false);
	gui.show();

	ofSetLogLevel(OF_LOG_VERBOSE);
}

void ShapeApp::setup() {


	ofSetFrameRate(30);
	TIME_SAMPLE_SET_FRAMERATE(30.0f);

	setupUI();

	// CPU
	view_f.init(DEPTH_X_RES, DEPTH_Y_RES);
	view_f.allocateHost();
	view_image.allocate(DEPTH_X_RES, DEPTH_Y_RES, OF_IMAGE_COLOR);

	// GPU
	view_f.allocateDevice();

	// voxel data
//=============================================================================
	// CPU
	min.set(-0.5, -0.5, -1.5);
	max.set(0.5, 0.5, -0.5);

	voxels.min = min;
	voxels.side_n = 256;
	voxels.size = (max - min) / (float) voxels.side_n;

	voxels.array_size = voxels.side_n * voxels.side_n * voxels.side_n;
	voxels.bytes_n = sizeof(float) * voxels.array_size;
	voxels.data = (float *) malloc(voxels.bytes_n);

	voxels.w_bytes_n = sizeof(unsigned char) * voxels.array_size;
	voxels.w_data = (unsigned char *) malloc(voxels.w_bytes_n);

	// GPU
	cudaMalloc((void **) &camera_opt.t, sizeof(float) * 16);
	cudaMalloc((void **) &camera_opt.it, sizeof(float) * 16);

	camera_opt.ref_pix_size = 0.1042;
	camera_opt.ref_distance = 120;

	cout << "camera_opt.ref_pix_size " << camera_opt.ref_pix_size << endl;
	cout << "camera_opt.ref_distance " << camera_opt.ref_distance << endl;

	setFloat3(camera_opt.min, min);
	setFloat3(camera_opt.max, max);

	cudaMalloc((void **) &dev_voxels.data, voxels.bytes_n);
	cudaMalloc((void **) &dev_voxels.w_data, voxels.w_bytes_n);
	setFloat3(&dev_voxels.min, voxels.min);
	setFloat3(&dev_voxels.size, voxels.size);

	dev_voxels.side_n = voxels.side_n;
	dev_voxels.side_n2 = dev_voxels.side_n * dev_voxels.side_n;
	dev_voxels.array_size = voxels.array_size;
	dev_voxels.bytes_n = voxels.bytes_n;
	dev_voxels.w_bytes_n = voxels.w_bytes_n;

	resetVoxels();
}

void setFloat3(float3 *f, ofPoint p) {
	f->x = p.x;
	f->y = p.y;
	f->z = p.z;
}

void setFloat3(float *f, ofPoint p) {
	f[0] = p.x;
	f[1] = p.y;
	f[2] = p.z;
}

void ShapeApp::exit() {

	gui.saveToXML();

	delete grab_cam;

	cudaFree(camera_opt.t);
	cudaFree(camera_opt.it);
	cudaFree(dev_voxels.data);
	cudaFree(dev_voxels.w_data);

	free(voxels.data);
	free(voxels.w_data);

	view_f.releaseHost();
	view_f.releaseDevice();

	cout << "ShapeApp::exit()" << endl;
}

