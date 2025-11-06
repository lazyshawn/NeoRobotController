
#include "interpolation/InterpSegment.h"


InterpConstraint::InterpConstraint(double vmax_, double vmin_, double amax_, double amin_, double jmax_, double jmin_) {
	vmax = vmax_;
	vmin = vmin_;
	amax = amax_;
	amin = amin_;
	jmax = jmax_;
	jmin = jmin_;
}

int InterpSegment::set_constraint(const InterpConstraint& constraint_) {
	return 0;
}
