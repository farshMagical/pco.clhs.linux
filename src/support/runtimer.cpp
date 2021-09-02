#include <runtimer.h>

using namespace std;

Timer::Timer() : m_beg(clock_t::now())
{
}

Timer::Timer(string name) : m_beg(clock_t::now())
{
	this->name = name;
}

void Timer::reset()
{
	m_beg = clock_t::now();
}

void Timer::reset(string name)
{
	m_beg = clock_t::now();
	this->name = name;
}

double Timer::elapsed() const
{
	return std::chrono::duration_cast<second_t>(clock_t::now() - m_beg).count();
}

std::ostream& operator<<(std::ostream& out, const Timer &t){
	out << left << setw(35) << "runTimer " + t.name << t.elapsed() << " sec";
	return out;
}
