/**
 * File:    random_colors.h
 *
 * Date:    07.04.2021
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_EXAMPLES_DRAWING_RANDOM_COLORS_H_
#define TRIVIS_EXAMPLES_DRAWING_RANDOM_COLORS_H_

#include "drawing/colors.h"

#include <vector>

namespace trivis_examples::drawing {

std::vector<RGB> RandomColors(int n, int seed = 42);

}

#endif //TRIVIS_EXAMPLES_DRAWING_RANDOM_COLORS_H_
