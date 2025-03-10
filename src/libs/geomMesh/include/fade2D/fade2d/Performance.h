// (c) 2010 Geom e.U. Bernhard Kornberger, Graz/Austria. All rights reserved.
//
// This file is part of the Fade2D library. You can use it for your personal
// non-commercial research. Licensees holding a commercial license may use this 
// file in accordance with the Commercial License Agreement provided 
// with the Software.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING 
// THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are not clear 
// to you.
// 
// Author: Bernhard Kornberger, bkorn (at) draw.at
// http://www.geom.at


#pragma once
#include <string>


#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE 
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined 
#endif 


/**  \brief Timer
*
* Call the timer function with a certain string to start time 
* measurement. Call it a second time with the same string to 
* finish the time measurement.
*
* @return -1 when the timer is started or the elapsed time in
* seconds when the timer is stopped. 
*/
CLASS_DECLSPEC
double timer(const std::string& str);

} // (namespace)
