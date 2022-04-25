/**
 * File:    random_colors.cc
 *
 * Date:    07.04.2021
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#include "drawing/random_colors.h"

#include <random>

using namespace trivis_examples::drawing;

std::vector<RGB> trivis_examples::drawing::RandomColors(int n, int seed) {
  std::mt19937 g(seed);
  std::uniform_int_distribution<int> d(0, 255);
  std::vector<RGB> random_colors(n);
  for (int i = 0; i < n; ++i) random_colors[i] = {d(g), d(g), d(g)};
  return random_colors;
}