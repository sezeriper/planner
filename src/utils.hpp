#pragma once

#include <shared_mutex>
#include <mutex>

namespace rota {
template<class T> 
struct mutex_guarded {
    using M = std::shared_mutex;
    using WL = std::unique_lock<M>;
    using RL = std::shared_lock<M>;

    auto read( auto f ) const {
        auto l = lock();
        return f(t);
    }
    auto write( auto f ) {
        auto l = lock();
        return f(t);
    }
    mutex_guarded()=default;
    explicit mutex_guarded(T in) : t(std::move(in)) {}
private:
    mutable M m;
    T t;
    auto lock() const { return RL(m); }
    auto lock() { return WL(m); }
};
}