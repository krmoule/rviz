/*
 * Copyright (c) 2014, Christie Digital Systems, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#include <ros/ros.h>

#include <OgreCamera.h>

#include "rviz/properties/float_property.h"
#include "rviz/default_plugin/view_controllers/christie_view_controller.h"

namespace {

// degree to radian
float D2R( float degree )
{
	return (degree/180.0f)*(float)M_PI;
}

// mimic glFrustim functionality
Ogre::Matrix4 frustum( float l, float r, float t, float b, float n, float f )
{
	return Ogre::Matrix4(
		2*n/(r-l),         0,  (r+l)/(r-l),            0,
			0, 2*n/(t-b),  (t+b)/(t-b),            0,
			0,         0, -(f+n)/(f-n), -2*f*n/(f-n),
			0,         0,           -1,            0
	 );
}

// Using angles (in degrees) for the left, right, top and bottom extents, build a custom projection
// matrix that can be given to an ogre camera
Ogre::Matrix4 goAutoCalToOgreProjectionMatrix( float l, float r, float t, float b, float ZNEAR, float ZFAR )
{
	float l_ = ZNEAR*tan( D2R( l ) );
	float r_ = ZNEAR*tan( D2R( r ) );
	float t_ = ZNEAR*tan( D2R( t ) );
	float b_ = ZNEAR*tan( D2R( b ) );

	return frustum( l_, r_, t_, b_, ZNEAR, ZFAR );
}

}

namespace rviz
{

ChristieViewController::ChristieViewController()
{
  position_x_property_ = new FloatProperty( "X", 0.0f, "Position in space, X.", this );
  position_y_property_ = new FloatProperty( "Y", 0.0f, "Position in space, Y.", this );
  position_z_property_ = new FloatProperty( "Z", 0.0f, "Position in space, Z.", this );

  orientation_y_property_ = new FloatProperty( "Y", 0.0f, "Orientation in space, Yaw.", this );
  orientation_p_property_ = new FloatProperty( "P", 0.0f, "Orientation in space, Pitch.", this );
  orientation_r_property_ = new FloatProperty( "R", 0.0f, "Orientation in space, Roll.", this );

  frustum_l_property_ = new FloatProperty( "Left", -20.0f, "Angle defining the frustu,m, Left", this );
  frustum_r_property_ = new FloatProperty( "Right", 20.0f, "Angle defining the frustu,m, Right", this );
  frustum_t_property_ = new FloatProperty( "Top", -20.0f, "Angle defining the frustu,m, Top", this );
  frustum_b_property_ = new FloatProperty( "Bottom", 20.0f, "Angle defining the frustu,m, Bottom", this );
}

ChristieViewController::~ChristieViewController()
{
}

void ChristieViewController::reset()
{
	position_x_property_->setFloat( 0.0f );
	position_y_property_->setFloat( 0.0f );
	position_z_property_->setFloat( 0.0f );

	orientation_y_property_->setFloat( 0.0f );
	orientation_p_property_->setFloat( 0.0f );
	orientation_r_property_->setFloat( 0.0f );

	frustum_l_property_->setFloat( -20.0f );
	frustum_r_property_->setFloat(  20.0f );
	frustum_t_property_->setFloat( -20.0f );
	frustum_b_property_->setFloat(  20.0f );
}

void ChristieViewController::update( float dt, float ros_dt )
{
	float x = position_x_property_->getFloat();
	float y = position_y_property_->getFloat();
	float z = position_z_property_->getFloat();

	float yaw = orientation_y_property_->getFloat();
	float pitch = orientation_p_property_->getFloat();
	float roll = orientation_r_property_->getFloat();

	float left = frustum_l_property_->getFloat();
	float right = frustum_r_property_->getFloat();
	float top = frustum_t_property_->getFloat();
	float bottom = frustum_b_property_->getFloat();

	camera_->setPosition( x, y, z );

	// TODO: this need to be replaced with a matrix built from yaw/pitch/roll
	camera_->lookAt( 0, 0, 0 );

	float near = camera_->getNearClipDistance();
	float far = camera_->getFarClipDistance();

	Ogre::Matrix4 projMat = goAutoCalToOgreProjectionMatrix( left, right, top, bottom, near, far );
	camera_->setCustomProjectionMatrix( true, projMat );
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::ChristieViewController, rviz::ViewController )
