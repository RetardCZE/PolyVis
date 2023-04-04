
#include <float.h>

/**
 * Different sources define the "machine epsilon" in one of two ways.
 * The C standard library header `float.h` defines DBL_EPSILON, such that
 *     1.0 + DBL_EPSILON > 1.0
 * in double-precision arithmetic, but such that
 *     1.0 + z = 1.0
 * for any `z < DBL_EPSILON`. Many other sources, included Shewchuk's original
 * code and the supporting paper, define the machine epsilon instead as the
 * largest floating-point number such that
 *     1.0 + epsilon = 1.0
 * The two are related by
 *     epsilon = DBL_EPSILON / 2.
 */
#define EPS (DBL_EPSILON / 2)

const double kEpsilon = EPS;
const double kSplitter = (1 << ((DBL_MANT_DIG + 1) >> 1)) + 1.0;

const double kResulterrbound = (3.0 + 8.0 * EPS) * EPS;

const double kCcwerrboundA = (3.0 + 16.0 * EPS) * EPS;
const double kCcwerrboundB = (2.0 + 12.0 * EPS) * EPS;
const double kCcwerrboundC = (9.0 + 64.0 * EPS) * EPS * EPS;
const double kO3DerrboundA = (7.0 + 56.0 * EPS) * EPS;
const double kO3DerrboundB = (3.0 + 28.0 * EPS) * EPS;
const double kO3DerrboundC = (26.0 + 288.0 * EPS) * EPS * EPS;
const double kIccerrboundA = (10.0 + 96.0 * EPS) * EPS;
const double kIccerrboundB = (4.0 + 48.0 * EPS) * EPS;
const double kIccerrboundC = (44.0 + 576.0 * EPS) * EPS * EPS;
const double kIsperrboundA = (16.0 + 224.0 * EPS) * EPS;
const double kIsperrboundB = (5.0 + 72.0 * EPS) * EPS;
const double kIsperrboundC = (71.0 + 1408.0 * EPS) * EPS * EPS;
