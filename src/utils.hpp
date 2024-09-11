#pragma once

#include <shared_mutex>

namespace rota {
template<
    class T,
    class M=std::shared_mutex,
    template<class...> class WL=std::unique_lock,
    template<class...> class RL=std::shared_lock
> 
struct mutex_guarded {
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
    auto lock() const { return RL<M>(m); }
    auto lock() { return WL<M>(m); }
};
}