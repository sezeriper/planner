#pragma once

#include "math.hpp"

#include <span>

namespace rota {

template <typename data_t>
class grid {
public:
    using size_t = std::uint64_t;

    struct cell {
        data_t* data;
        size_t* size;
    };

    grid(size_t width, size_t height, size_t capacity):
        _width(width),
        _height(height),
        _capacity(capacity),
        _data(std::make_unique<data_t[]>(width * height * capacity)),
        _size(std::make_unique<size_t[]>(width * height))
    {}

    cell get_cell(size_t x, size_t y) const {
        return {
            &_data[(y * _width * _capacity) + (x * _capacity)],
            &_size[(y * _width) + x]
        };
    }

    data_t* add_data(size_t x, size_t y, const data_t& data) {

        auto cell = get_cell(x, y);

        // if (*cell.size < _capacity) {
            cell.data[*cell.size] = data;
            ++(*cell.size);

            return cell.data + *cell.size - 1;
        // } else {
        //     std::cerr << "Cell is full" << std::endl;
        // }
    }

    size_t get_width() const {
        return _width;
    }

    size_t get_height() const {
        return _height;
    }

    std::span<data_t> get_data() const {
        return {_data.get(), _width * _height * _capacity};
    }

    size_t get_capacity() const {
        return _capacity;
    }

private:
    size_t _width;
    size_t _height;
    size_t _capacity;

    std::unique_ptr<data_t[]> _data;
    std::unique_ptr<size_t[]> _size;
};

template <typename data_t>
class grid_spatial : public grid<data_t> {
public:
    using typename grid<data_t>::cell;

    struct coord {
        size_t x;
        size_t y;
    };

    grid_spatial() = delete;

    grid_spatial(point top_left, point bottom_right, real_t cell_size, size_t capacity) :
        grid<data_t>(
            ((bottom_right.x - top_left.x) / cell_size) + 1,
            ((bottom_right.y - top_left.y) / cell_size) + 1,
            capacity
        ),
        _top_left(top_left),
        _bottom_right(bottom_right),
        _cell_size(cell_size)
    {}

    constexpr coord get_coord(point p) const {
        size_t x = static_cast<size_t>((p.x - _top_left.x) / _cell_size);
        size_t y = static_cast<size_t>((p.y - _top_left.y) / _cell_size);
        return {x, y};
    }

    using grid<data_t>::get_cell;
    cell get_cell(point p) const {
        coord c = get_coord(p);
        return get_cell(c.x, c.y);
    }

    using grid<data_t>::add_data;
    const data_t* add_data(point p, const data_t& data) {
        coord c = get_coord(p);
        return add_data(c.x, c.y, data);
    }


private:
    point _top_left;
    point _bottom_right;
    real_t _cell_size;
};
}