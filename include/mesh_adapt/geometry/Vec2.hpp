#pragma once
#include <cmath>
#include <iostream>

namespace mesh_adapt {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double _x, double _y) : x(_x), y(_y) {}

    //~ Vec2 operator+(const Vec2& b) const {
        //~ return {x + b.x, y + b.y};
    //~ }

    //~ Vec2 operator-(const Vec2& b) const {
        //~ return {x - b.x, y - b.y};
    //~ }

    //~ Vec2 operator*(double s) const {
        //~ return {x * s, y * s};
    //~ }

    double dot(const Vec2& b) const {
        return x*b.x + y*b.y;
    }

    double norm() const {
        return std::sqrt(x*x + y*y);
    }

    // Distancia a otro Vec2
    double distance(const Vec2& o) const {
        double dx = x - o.x;
        double dy = y - o.y;
        return std::sqrt(dx*dx + dy*dy);
    }


    bool operator==(const Vec2& o) const {
        constexpr double tol = 1e-12;
        return std::abs(x - o.x) < tol && std::abs(y - o.y) < tol;
    }

    bool operator!=(const Vec2& o) const { return !(*this == o); }

    Vec2 operator/(double s) const {
        return {x / s, y / s};
    }
        
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) {
    return Vec2(a.x + b.x, a.y + b.y);
}

inline Vec2 operator-(const Vec2& a, const Vec2& b) {
    return Vec2(a.x - b.x, a.y - b.y);
}

inline Vec2 operator*(double s, const Vec2& v) {
    return Vec2(s * v.x, s * v.y);
}

inline Vec2 operator*(const Vec2& v, double s) {
    return Vec2(s * v.x, s * v.y);
}

inline double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

inline double norm(const Vec2& v) {
    return std::sqrt(v.x*v.x + v.y*v.y);
}

inline double distance(const Vec2& a, const Vec2& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

inline std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

} // namespace mesh_adapt

