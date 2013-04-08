#ifndef _AND_DETECTOR_
#define _AND_DETECTOR_

namespace ROSRobot
{
	template<class A, class B>
	class AndDetector
	{
	private:
		A& _a;
		B& _b;
	public: /*Constructors*/
		AndDetector(A&, B&);
	
	public:
		bool operator()(void);
	};

	template<class A, class B>
	AndDetector<A, B>::AndDetector(A& a, B& b)
		: _a(a), _b(b)
	{
	}

	template<class A, class B>
	bool AndDetector<A, B>::operator()(void)
	{
		return this->_a() && this->_b();
	}
}

#endif
