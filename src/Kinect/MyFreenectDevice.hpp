/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
*/

#include "libfreenect.hpp"
#include "Mutex.hpp"
#include <vector>

#ifndef _MY_FREENECT_DEVICE_
#define _MY_FREENECT_DEVICE_
class MyFreenectDevice : public Freenect::FreenectDevice 
{
private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_video;
	std::vector<uint16_t> m_gamma;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;

public:
	MyFreenectDevice(freenect_context*, int);

	~MyFreenectDevice(void);

	// Do not call directly even in child
	void VideoCallback(void*, uint32_t);

	// Do not call directly even in child
	void DepthCallback(void*, uint32_t);
private:	
	bool getRGB(std::vector<uint8_t> &buffer);

	bool getDepth(std::vector<uint8_t> &buffer);
public:
	std::vector<uint8_t> myGet();
};
#endif
