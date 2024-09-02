#pragma once

#include "math.hpp"

#include <span>
#include <memory>

namespace rota {

template <typename data_t>
class grid {
public:
    struct cell {
        data_t* data;
        size_t* size;
    };

    grid(int width, int height, int capacity):
        _width(width),
        _height(height),
        _capacity(capacity),
        _data(std::make_unique<data_t[]>(width * height * capacity)),
        _size(std::make_unique<size_t[]>(width * height))
    {}

    cell get_cell(int x, int y) const {
        return {
            &_data[(y * _width * _capacity) + (x * _capacity)],
            &_size[(y * _width) + x]
        };
    }

    data_t& add_data(size_t x, size_t y, const data_t& data) {
        auto cell = get_cell(x, y);

        cell.data[*cell.size] = std::move(data);
        ++(*cell.size);

        return *(cell.data + *cell.size - 1);
    }

    size_t get_width() const {
        return _width;
    }

    size_t get_height() const {
        return _height;
    }

    const std::span<data_t> get_data() const {
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
    struct coord {
        size_t x;
        size_t y;
    };

    grid_spatial() = delete;

    grid_spatial(vec2_t top_left, vec2_t bottom_right, real_t cell_size, int capacity) :
        grid<data_t>(
            ((bottom_right.x - top_left.x) / cell_size) + 1,
            ((bottom_right.y - top_left.y) / cell_size) + 1,
            capacity
        ),
        _top_left(top_left),
        _bottom_right(bottom_right),
        _cell_size(cell_size)
    {}

    constexpr real_t get_cell_size() const {
        return _cell_size;
    }

    constexpr vec2_t get_top_left() const {
        return _top_left;
    }

    constexpr vec2_t get_bottom_right() const {
        return _bottom_right;
    }

    constexpr coord get_coord(vec2_t p) const {
        size_t x = (p.x - _top_left.x) / _cell_size;
        size_t y = (p.y - _top_left.y) / _cell_size;
        return {x, y};
    }

    using grid<data_t>::get_cell;
    grid<data_t>::cell get_cell(vec2_t p) const {
        coord c = get_coord(p);
        return grid<data_t>::get_cell(c.x, c.y);
    }

    std::vector<typename grid<data_t>::cell> get_cells_in_radius(vec2_t p, real_t radius) const {
        std::vector<typename grid<data_t>::cell> cells;

        const coord c = get_coord(p);
        const int c_x = c.x;
        const int c_y = c.y;
        const int radius_cells = std::ceil(radius / _cell_size);

        for (int y = c_y - radius_cells; y <= c_y + radius_cells; ++y) {
            for (int x = c_x - radius_cells; x <= c_x + radius_cells; ++x) {
                if (x < 0 || x >= grid<data_t>::get_width() ||
                    y < 0 || y >= grid<data_t>::get_height()) {
                    continue;
                }

                cells.emplace_back(grid<data_t>::get_cell(x, y));
            }
        }

        return cells;
    }

    using grid<data_t>::add_data;
    data_t& add_data(vec2_t p, const data_t& data) {
        coord c = get_coord(p);
        return grid<data_t>::add_data(c.x, c.y, data);
    }

private:
    vec2_t _top_left;
    vec2_t _bottom_right;
    real_t _cell_size;
};
}