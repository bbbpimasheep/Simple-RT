#include "bounds.h"

const Bounds3 Bounds3::Empty    = Bounds3(Interval::Empty,    Interval::Empty,    Interval::Empty);
const Bounds3 Bounds3::Universe = Bounds3(Interval::Universe, Interval::Universe, Interval::Universe);
