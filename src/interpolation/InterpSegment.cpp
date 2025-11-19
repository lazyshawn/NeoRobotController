
#include "interpolation/InterpSegment.h"


InterpConstraint::InterpConstraint(double vmax_, double vmin_, double amax_, double amin_, double jmax_, double jmin_) {
	vmax = vmax_;
	vmin = vmin_;
	amax = amax_;
	amin = amin_;
	jmax = jmax_;
	jmin = jmin_;
}



InterpBoundary::InterpBoundary(double q0_, double q1_, double v0_, double v1_, double a0_, double a1_) {
	qk = q0 = q0_;
	vk = v0 = v0_;
	ak = a0 = a0_;
	q1 = q1_;
	v1 = v1_;
	a1 = a1_;

	jk = 0.0;
}




int InterpSegment::set_constraint(const InterpConstraint& constraint_) {
	constraint = constraint_;

	return 0;
}


int InterpSegment::add_segment(const InterpBoundary& boundary) {
	boundaryQueue.push(boundary);

	return 0;
}

int InterpSegment::pop_front_segment(InterpBoundary& boundary) {
	if (boundaryQueue.empty())
		return 1;

	boundary = boundaryQueue.front();

	boundaryQueue.pop();

	return 0;
}
