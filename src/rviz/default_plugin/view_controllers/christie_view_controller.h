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

#ifndef RVIZ_CHRISTIE_VIEW_CONTROLLER_H
#define RVIZ_CHRISTIE_VIEW_CONTROLLER_H

#include "rviz/view_controller.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

/**
 * \brief A view controller that defines parameters compatible with AutoCal
 */
class ChristieViewController : public ViewController
{
Q_OBJECT
public:
	ChristieViewController();
	virtual ~ChristieViewController();

	virtual void reset();

protected:
	virtual void update( float dt, float ros_dt );

private:
	FloatProperty* position_x_property_;
	FloatProperty* position_y_property_;
	FloatProperty* position_z_property_;

	FloatProperty* orientation_y_property_;
	FloatProperty* orientation_p_property_;
	FloatProperty* orientation_r_property_;

	FloatProperty* frustum_l_property_;
	FloatProperty* frustum_r_property_;
	FloatProperty* frustum_t_property_;
	FloatProperty* frustum_b_property_;
};

}

#endif /* RVIZ_CHRISTIE_VIEW_CONTROLLER_H */
