#ifndef BIRD_H
#define BIRD_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"

using namespace std;

namespace CGL {

	struct Bird {
		Bird(PointMass* a) : pm_a(a) {
		}

		PointMass* pm_a;
	};
}
#endif /* Bird_H */