#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <iomanip>
class Timer
{
public:
	Timer();
	Timer(std::string name);
	void reset();
	void reset(std::string name);
	double elapsed() const;
	friend std::ostream& operator<<(std::ostream& out, const Timer &t);

private:
	std::string name = "-";
	using clock_t = std::chrono::high_resolution_clock;
	using second_t = std::chrono::duration<double, std::ratio<1> >;

	std::chrono::time_point<clock_t> m_beg;
};
