#include "speedsample.h"

#include <iostream>

SpeedSample::SpeedSample(const size_t length) :
	m_length(length),
	m_lastTime(now())
{

}

SpeedSample::~SpeedSample() {

}

double SpeedSample::getSpeed() const {
	double itemSum = 0, timeSum = 0;
	auto it1 = items.begin();
	auto it2 = time.begin();
	while (it1 != items.end()) {
		itemSum += *it1;
		timeSum += *it2;
		it1++;
		it2++;
	}

	return (1000 * itemSum) / timeSum;
}

void SpeedSample::sample(const double V) {
	const timepoint newTime = now();
	auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(newTime - m_lastTime).count();
	//std::cout << V << " " << delta << " \n";
	items.push_back(V);
	time.push_back(delta);
	m_lastTime = newTime;
	if (items.size() > m_length) {
		items.pop_front();
		time.pop_front();
	}
}

SpeedSample::timepoint SpeedSample::now() {
	return std::chrono::steady_clock::now();
}
