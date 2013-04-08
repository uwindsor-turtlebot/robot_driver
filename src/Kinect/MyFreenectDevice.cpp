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


#include "MyFreenectDevice.hpp"
#include "Mutex.hpp"
#include <vector>

MyFreenectDevice::MyFreenectDevice(freenect_context *_ctx, int _index)
	: Freenect::FreenectDevice(_ctx, _index), 
	m_buffer_depth(	freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
	m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), 
	m_gamma(2048), 
	m_new_rgb_frame(false),
 	m_new_depth_frame(false)
{
	for( unsigned int i = 0 ; i < 2048 ; i++) 
	{
		float v = i/2048.0;
		v = v * v * v * 6;
		m_gamma[i] = v*6*256;
	}
}

MyFreenectDevice::~MyFreenectDevice(void)
{

}

// Do not call directly even in child
void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) 
{
	Mutex::ScopedLock lock(m_rgb_mutex);
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
	m_new_rgb_frame = true;
}


// Do not call directly even in child
void MyFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) 
{
	Mutex::ScopedLock lock(m_depth_mutex);
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	for( unsigned int i = 0 ; i < 640*480 ; i++) 
	{
		int pval = m_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) 
		{
			case 0:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			case 1:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = lb;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 2:
				m_buffer_depth[3*i+0] = 255-lb;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 3:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = lb;
				break;
			case 4:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255;
				break;
			case 5:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			default:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 0;
				break;
		}
	}
	m_new_depth_frame = true;
}


bool MyFreenectDevice::getRGB(std::vector<uint8_t> &buffer) 
{
	Mutex::ScopedLock lock(m_rgb_mutex);
	if (!m_new_rgb_frame)
	{
		return false;
	}
	buffer.swap(m_buffer_video);
	m_new_rgb_frame = false;
	return true;
}


bool MyFreenectDevice::getDepth(std::vector<uint8_t> &buffer) 
{
	Mutex::ScopedLock lock(m_depth_mutex);
	if (!m_new_depth_frame)
	{
		return false;
	}
	buffer.swap(m_buffer_depth);
	m_new_depth_frame = false;
	return true;
}


std::vector<uint8_t> MyFreenectDevice::myGet() 
{
	return m_buffer_depth;
}
