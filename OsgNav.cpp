/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * VR Juggler is (C) Copyright 1998-2010 by Iowa State University
 *
 * Original Authors:
 *   Allen Bierbaum, Christopher Just,
 *   Patrick Hartling, Kevin Meinert,
 *   Carolina Cruz-Neira, Albert Baker
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include <vrj/vrjConfig.h>

#include "OsgNav.h"

#include <osg/Math>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Vec3>
#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>

#include <gmtl/Vec.h>
#include <gmtl/Coord.h>
#include <gmtl/Xforms.h>
#include <gmtl/Math.h>

#ifdef USE_REMOTE_NAV
#   include <remotenav/Subject/RemoteNavSubjectImpl.h>
#endif

OsgNav::OsgNav(vrj::Kernel* kern, int& argc, char** argv)
	: vrj::osg::App(kern) {
	mFileToLoad = std::string("");
}

void OsgNav::latePreFrame() {
	gmtl::Matrix44f world_transform;
	gmtl::invertFull(world_transform, mNavigator.getCurPos());
	// Update the scene graph
	osg::Matrix osg_current_matrix;
	osg_current_matrix.set(world_transform.getData());
	mNavTrans->setMatrix(osg_current_matrix);

	// Finish updating the scene graph.
	vrj::osg::App::latePreFrame();
}

void OsgNav::preFrame() {
	vpr::Interval cur_time = mWand->getTimeStamp();
	vpr::Interval diff_time(cur_time - mLastPreFrameTime);
	if (mLastPreFrameTime.getBaseVal() >= cur_time.getBaseVal()) {
		diff_time.secf(0.0f);
	}

	float time_delta = diff_time.secf();
	mLastPreFrameTime = cur_time;

	// Get wand data
	gmtl::Matrix44f wandMatrix = mWand->getData();      // Get the wand matrix

	// If we are pressing button 1 then translate in the direction the wand is
	// pointing.
	if (mButton0->getData() == gadget::Digital::ON) {
		gmtl::Vec3f direction;
		gmtl::Vec3f Zdir = gmtl::Vec3f(0.0f, 0.0f, -10.0f);
		gmtl::xform(direction, wandMatrix, Zdir);

		mNavigator.setVelocity(direction);
	}  // Make sure to reset the velocity when we stop pressing the button.
	else if (mButton0->getData() == gadget::Digital::TOGGLE_OFF) {
		mNavigator.setVelocity(gmtl::Vec3f(0.0, 0.0, 0.0));
	}

	if (mButton1->getData() == gadget::Digital::TOGGLE_ON) {
		std::cout << "Cur Pos: " << mNavigator.getCurPos() << std::endl;
	}

	// If we are pressing button 2 then rotate in the direction the wand is
	// pointing.
	if (mButton2->getData() == gadget::Digital::ON) {
		// Only allow Yaw (rot y)
		gmtl::EulerAngleXYZf euler(0.0f, gmtl::makeYRot(mWand->getData()), 0.0f);
		gmtl::Matrix44f rot_mat = gmtl::makeRot<gmtl::Matrix44f>(euler);
		mNavigator.setRotationalVelocity(rot_mat);
	}
	// Make sure to reset the rotational velocity when we stop pressing the
	// button.
	else if (mButton2->getData() == gadget::Digital::TOGGLE_OFF) {
		mNavigator.setRotationalVelocity(gmtl::Matrix44f());
	}
	// Update the navigation using the time delta between
	mNavigator.update(time_delta);
}

void OsgNav::initScene() {
	// Initialize devices
	const std::string wand("VJWand");
	const std::string vjhead("VJHead");
	const std::string but0("VJButton0");
	const std::string but1("VJButton1");
	const std::string but2("VJButton2");

	mWand.init(wand);
	mHead.init(vjhead);
	mButton0.init(but0);
	mButton1.init(but1);
	mButton2.init(but2);

	myInit();
}

void OsgNav::myInit() {
	//
	//          /-- mNoNav
	// mRootNode
	//         \-- mNavTrans -- mModelTrans -- mModel

	//The top level nodes of the tree
	mRootNode = new osg::Group();
	mNoNav    = new osg::Group();
	mNavTrans = new osg::MatrixTransform();


	mNavigator.init();

	mRootNode->addChild(mNoNav.get());
	mRootNode->addChild(mNavTrans.get());

	//Load the model
	std::cout << "Attempting to load file: " << mFileToLoad << "... ";
	mModel = osgDB::readNodeFile(mFileToLoad);
	std::cout << "done." << std::endl;

	// Transform node for the model
	mModelTrans  = new osg::MatrixTransform();
	//This can be used if the model orientation needs to change
	mModelTrans->preMult(osg::Matrix::rotate(gmtl::Math::deg2Rad(-90.0f),
	                     1.0f, 0.0f, 0.0f));

	if (! mModel.valid()) {
		std::cout << "ERROR: Could not load file: " << mFileToLoad << std::endl;
	} else {
		// Add model to the transform
		mModelTrans->addChild(mModel.get());
	}

	// Add the transform to the tree
	mNavTrans->addChild(mModelTrans.get());

}
