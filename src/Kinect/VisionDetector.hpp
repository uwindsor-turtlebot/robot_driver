#include "MyFreenectDevice.hpp"

#ifndef _VISION_DETECTOR_
#define _VISION_DETECTOR_

class VisionDetector 
{  
private:
	int _depth_threshold;
	float _percentage_threshold;
	MyFreenectDevice& _freenect_device;
	
public: /*Constructors*/
	VisionDetector(MyFreenectDevice&);
	VisionDetector(MyFreenectDevice&, int, float);
	~VisionDetector();
	
public: /*Accessors*/
	int get_depth_threshold() const;
	float get_percentage_threshold() const;
	MyFreenectDevice& get_device() const;
		
public:
	bool operator()(void) const;	
	bool checkPixel(uint8_t, uint8_t, uint8_t) const;
};
#endif
