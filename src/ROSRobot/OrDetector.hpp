
#ifndef _OR_DETECTOR_
#define _OR_DETECTOR_

namespace ROSRobot
{
	template<class A, class B>
	class OrDetector
	{
	private:
		A& _a;
		B& _b;
	public: /*Constructors*/
		OrDetector(A&, B&);
	
	public:
		bool operator()(void);
	};

	template<class A, class B>
	OrDetector<A, B>::OrDetector(A& a, B& b)
		: _a(a), _b(b)
	{
	}

	template<class A, class B>
	bool OrDetector<A, B>::operator()(void)
	{
		return this->_a() || this->_b();
	}
}

#endif
