#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxGrabCam.h"
#include "ofxSimpleGuiToo.h"
#include "ofxTimeMeasurements.h"

#include "Utils.h"
#include "Frame.h"
#include "Voxels.h"
#include "Settings.h"

#include "VoxelsCUDA.h"
#include "ofxRangeToWorldCUDA.h"

#include "libfreenect-registration.h"

#include <iostream>
#undef Success // fix for Eigen and X11 defines conflict
#include <Eigen/Dense>

#include <cuda_runtime.h>
#include <helper_cuda.h>    // includes cuda.h and cuda_runtime_api.h

// simple utils for CUDA code
void setFloat3(float3 *f, ofPoint p);
void setFloat3(float *f, ofPoint p);

void drawCameraPose(ofxKinect *kinect,
		ofColor color, ofMatrix4x4 transform_matrix);

void rayMarchCUDA(CameraOptions camera_opt, float march_step,
		float march_iterations_n, int step, VoxelVolumeCUDA voxels,
		FrameDataCUDA data);

class ShapeApp : public ofBaseApp {
//class ShapeApp : public ofxFensterListener {
public:

	Frame view_f; // estimated from voxels for surface viewing

	CameraOptions camera_opt;

	// voxels on CPU
	VoxelVolume voxels;

	// voxels on GPU
	VoxelVolumeCUDA dev_voxels;

	ofImage view_image;

//=============================================================================

	void updateVoxels();
	void updateVoxelsGame(VoxelVolume *voxels);
	void updateVoxelsTest(VoxelVolume *voxels);

	void resetVoxels();
	void resetVoxelWeights();
	void rayMarch(Frame *f, ofMatrix4x4 *t);

	void drawMap(ofImage *image, Frame *f,
			ofPoint offset, float scale, bool flip);

	// visualization
//=============================================================================
	void drawVoxels();
	void drawVolume();

	bool ui_show_depth;

	// rays
	int ui_rays_step;
	float ui_image_scale;
	float ui_march_step;
	int ui_march_iterations_n;

	// voxels
	bool ui_update_voxels;
	bool ui_estimate_maps;
	bool ui_reset_voxels;
	bool ui_reset_voxel_weights;

	// thresholds for the depth data
	// use only the points inside of the 3d box
	ofPoint min;
	ofPoint max;

	ofxGrabCam *grab_cam;

	void setupUI();

//============================================================================
	void setup();
	void update();
	void draw();

	void exit();

	void keyPressed(ofKeyEventArgs &args);
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

};
