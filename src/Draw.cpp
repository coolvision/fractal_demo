/*
 * Draw.cpp
 *
 *  Created on: 2012
 *      Author: sk
 */

#include "ShapeApp.h"
#include <iostream>

// used for current surface estimate at tracking
// and for surface preview
void ShapeApp::rayMarch(Frame *f, ofMatrix4x4 *t) {

	copyToDevice(camera_opt.t, t->getPtr(), sizeof(float) * 16);

	rayMarchCUDA(camera_opt, ui_march_step, ui_march_iterations_n, ui_rays_step,
			dev_voxels, f->dev);
	sync();

	copyFromDevice(f->host.normals, f->dev.normals, f->points_bn);
	copyFromDevice(f->host.points, f->dev.points, f->points_bn);
	copyFromDevice(f->host.depth, f->dev.depth, f->dev.depth_bn);
}

void ShapeApp::updateVoxels() {

	copyToDevice(dev_voxels.data, voxels.data, dev_voxels.bytes_n);
	sync();
}

void ShapeApp::resetVoxels() {

	for (int i = 0; i < voxels.array_size; i++) {
		voxels.data[i] = SDF_NONE;
		voxels.w_data[i] = 0;
	}

	copyToDevice(dev_voxels.data, voxels.data, dev_voxels.bytes_n);
	copyToDevice(dev_voxels.w_data, voxels.w_data, dev_voxels.w_bytes_n);

	sync();
}

void ShapeApp::resetVoxelWeights() {

	for (int i = 0; i < voxels.array_size; i++) {
		voxels.w_data[i] = 0;
	}

	copyToDevice(dev_voxels.w_data, voxels.w_data, dev_voxels.w_bytes_n);

	sync();
}

#define SIDEBAR_WIDTH 250

void setVoxel(VoxelVolume *voxels, ofPoint p, float v) {

	static int x_step = voxels->side_n * voxels->side_n;
	static int y_step = voxels->side_n;
	static int max_voxel_n = voxels->side_n * voxels->side_n * voxels->side_n;

	ofPoint voxel_p = p - voxels->min;
	voxel_p /= voxels->size;

	int vx = (int)voxel_p.x;
	int vy = (int)voxel_p.y;
	int vz = (int)voxel_p.z;

	unsigned int index = vx * x_step + vy * y_step + vz;
	if (index < max_voxel_n) {
		voxels->data[index] = v;
	}
}

void ShapeApp::updateVoxelsGame(VoxelVolume *voxels) {

	static ofPoint v[4] = {
			ofPoint(max.x, max.y, max.z),
			ofPoint(-max.x, - max.y, max.z),
			ofPoint(-max.x, max.y, -max.z),
			ofPoint(max.x, - max.y, -max.z),
	};

	//	start with a random point inside of a volume
	static ofPoint start(ofRandom(min.x, max.x),
						 ofRandom(min.y, max.y),
						 ofRandom(min.z, max.z));

	// run for some number of iterations
	for (int i = 0; i < 100; i++) {

		// select random vertex
		int v_i = (int)ofRandom(0.0f, 4.0f);

		// move half-way to this point
		start = (start + v[v_i]) / 2.0f;

		// find out what the voxel coord
		setVoxel(voxels, start, 0.1f);
	}
}

void ShapeApp::updateVoxelsTest(VoxelVolume *voxels) {

	float radius = 0.3;

	int x_step = voxels->side_n * voxels->side_n;
	int y_step = voxels->side_n;

	// this are world coords
	ofPoint voxel_coord;
	ofPoint voxel_cube_center = (max + min)/2;
	ofPoint box(0.4, 0.2, 0.1);
	ofPoint p;
	// now walk through all of the voxels
	for (int x = 0; x < voxels->side_n; x++) {
		for (int y = 0; y < voxels->side_n; y++) {

			float *v = &voxels->data[x * x_step + y * y_step];

			// to get the world coordinates,
			// take the start point
			// and add offsets
			voxel_coord.set(voxels->min.x + x * voxels->size.x,
					voxels->min.y + y * voxels->size.y,
					voxels->min.z);

			// inner loop is for memory aligned line of voxels
			for (int z = 0; z < voxels->side_n; z++) {

				p = voxel_coord - voxel_cube_center;

				float sphere_distance = p.length() - radius;

				ofPoint pa(abs(p.x), abs(p.y), abs(p.z));
				ofPoint pb = pa - box;
				ofPoint max(MAX(pb.x, 0.0), MAX(pb.y, 0.0), MAX(pb.z, 0.0));
				float box_distance = max.length();

				float distance = MIN(sphere_distance, box_distance);

				if (distance > -SDF_T && distance < SDF_T) {
					*v = distance;
				} else {
					*v = FLT_MAX;
				}

				v++; // memory
				voxel_coord.z += voxels->size.z;
			}
		}
	}
}

void ShapeApp::draw() {

	// shape update
//=============================================================================
//	static ofNode t_inc;
	static ofMatrix4x4 t_estimate;
//	static ofMatrix4x4 t_estimate_inv;
//	ofMatrix4x4 t;
//	ofMatrix4x4 it;

	updateVoxelsGame(&voxels);
	updateVoxels();

	if (ui_reset_voxels) {
		ui_reset_voxels = false;
		resetVoxels();
		resetVoxelWeights();
	}

	if (ui_reset_voxel_weights) {

		ui_reset_voxel_weights = false;
		resetVoxels();
		updateVoxelsTest(&voxels);
		updateVoxels();
	}

	// viewport and projection for 3d visualization
	ofViewport(SIDEBAR_WIDTH, 0, ofGetWindowWidth() - SIDEBAR_WIDTH,
			ofGetWindowHeight());

	grab_cam->begin();
	ofDrawAxis(500);
	drawVolume();
	grab_cam->end();

	ofViewport(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

	ofMatrix4x4 t = grab_cam->getLocalTransformMatrix();
	rayMarch(&view_f, &t);
	drawMap(&view_image, &view_f, ofPoint(SIDEBAR_WIDTH, 100), ui_image_scale,
			true);


	ofSetColor(ofColor::gray);
	ofRect(0, 0, SIDEBAR_WIDTH, ofGetWindowHeight());
	gui.draw();
	TIME_SAMPLE_DRAW(SIDEBAR_WIDTH, 50);

	usleep(3000);
}

void ShapeApp::drawMap(ofImage *image, Frame *f, ofPoint offset, float scale,
		bool flip) {

	unsigned char *pixels = image->getPixels();

	if (ui_show_depth) {
		float *d = f->host.depth;
		float depth = 0;
		unsigned char r, g, b;
		for (int i = 0; i < 640 * 480; i++) {
			depth = (*d);

			if (depth == 0.0f) {
				r = 0;
				g = 0;
				b = 0;
			} else if (depth < 512.0f) {
				r = 0;
				g = depth / 2;
				b = 0;
			} else if (depth < 1024.0f) {
				r = (depth - 512.0f) / 2.0f;
				g = 0;
				b = 0;
			} else {
				r = 0;
				g = 0;
				b = (depth - 1024.0f) / 2.0f;
			}

			(*pixels++) = r;
			(*pixels++) = g;
			(*pixels++) = b;

			d++;
		}
	} else {
		float *n = f->host.normals;
		for (int i = 0; i < 640 * 480; i++) {
			(*pixels++) = (unsigned char) abs((*n++) * 255.0);
			(*pixels++) = (unsigned char) abs((*n++) * 255.0);
			(*pixels++) = (unsigned char) abs((*n++) * 255.0);
		}
	}

	image->update();
	if (flip) {
		image->mirror(false, true);
	}

	ofSetColor(ofColor::white);
	image->draw(offset.x, offset.y, image->width * scale,
			image->height * scale);
}

