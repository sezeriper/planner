#pragma once

#include <string>
#include <chrono>
#include <iostream>


namespace rota {
class timer {
public:
    timer() = delete;
    timer(const std::string& name) :
        _name(name), _start(std::chrono::high_resolution_clock::now())
    {}

    ~timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - _start);
        std::cout << _name << " took " << duration.count() << "ms" << std::endl;
    }

private:
    std::string _name;
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
};
}
