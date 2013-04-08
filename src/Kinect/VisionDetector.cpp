#include "MyFreenectDevice.hpp"
#include "VisionDetector.hpp"

VisionDetector::VisionDetector(MyFreenectDevice& freenect_device) : 
	_depth_threshold(100), 
	_percentage_threshold(0.5),
	_freenect_device(freenect_device)
{
}

VisionDetector::VisionDetector(MyFreenectDevice& freenect_device, int d, float p) : 
	_depth_threshold(d), 
	_percentage_threshold(p),
	_freenect_device(freenect_device)
{
}

VisionDetector::~VisionDetector(void)
{
}

int VisionDetector::get_depth_threshold() const
{
	return this->_depth_threshold;
}

float VisionDetector::get_percentage_threshold() const
{
	return this->_percentage_threshold;
}

MyFreenectDevice& VisionDetector::get_device() const
{
	return this->_freenect_device;
}

bool VisionDetector::operator()(void) const
{
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	std::vector<uint8_t> depth = _freenect_device.myGet();
	int pixelstooclose = 0;
	int count = 0;
	float percent;

	for(int p = 0; p < 640*480 ; p++)
	{			
		byte1 = depth[3*p];
		byte2 = depth[3*p+1];
		byte3 = depth[3*p+2];

		if (this->checkPixel(byte1, byte2, byte3))
			pixelstooclose++;

		count++;	
	}

	percent = pixelstooclose;

	if(count == 0)
		percent = 1;
	else
		percent /= count;

	bool tooClose = percent > _percentage_threshold;

	if (tooClose)
	{
		_freenect_device.setLed(LED_GREEN);
		return true;
	}

	_freenect_device.setLed(LED_RED);
	return false;
}

bool VisionDetector::checkPixel(uint8_t byte1, uint8_t byte2, uint8_t byte3) const
{
	if (byte1 == 0 && byte2 == 0 && byte3 == 0) 
	{
		return true;		//pixels that cannot be detected (either too far or too close) are counted
	}
	if (byte1 == 255)
	{
		if (byte3 == 0)
		{
			return byte2 < _depth_threshold - 256;
		}
		else if (byte2 == byte3)
		{
			return 256 - byte2 < _depth_threshold;
		}
	}
	else if (byte1 == 0)
	{
		if (byte2 == 255)
		{
			return byte3 < _depth_threshold - 768;
		}
		else if (byte2 == 0)
		{
			return 1535 - byte3 < _depth_threshold;
		}
		else if (byte3 == 255)
		{
			return 1280 - byte2 < _depth_threshold;
		}
	}
	else
	{
		return 768 - byte1 < _depth_threshold;
	}
	return false;             
	
}
