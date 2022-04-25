/**
 * File:   grid.h
 *
 * Date:   24.03.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_CORE_BUCKETING_GRID_H_
#define TRIVIS_CORE_BUCKETING_GRID_H_

#include <vector>

namespace trivis::core::bucketing {

class Grid {

public:

    using Cell = std::vector<int>;

    Grid() = default;

    Grid(int nrow, int ncol) : _nrow(nrow), _ncol(ncol), _data(nrow * ncol) {}

    void Resize(int nrow, int ncol) {
        _nrow = nrow;
        _ncol = ncol;
        _data.resize(nrow * ncol);
    }

    [[nodiscard]] int nrow() const { return _nrow; }

    [[nodiscard]] int ncol() const { return _ncol; }

    [[nodiscard]] const Cell &operator()(int row, int col) const noexcept(true) { return _data[col * _ncol + row]; }

    [[nodiscard]] Cell &operator()(int row, int col) noexcept(true) { return _data[col * _ncol + row]; }

private:

    int _nrow = 0;
    int _ncol = 0;
    std::vector<Cell> _data;

};

}

#endif //TRIVIS_CORE_BUCKETING_GRID_H_
