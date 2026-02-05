#pragma once
#include <cmath>
#include <iostream>
//#include <numbers>  // Para std::numbers::pi en C++20

namespace mesh_adapt {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double _x, double _y) : x(_x), y(_y) {}

    double dot(const Vec2& b) const {
        return x*b.x + y*b.y;
    }

    double norm() const {
        return std::sqrt(x*x + y*y);
    }
    
    // NUEVO: Vector normalizado (vector unitario)
    Vec2 normalized() const {
        double n = norm();
        if (n == 0.0) return {0.0, 0.0};  // Evitar división por cero
        return {x / n, y / n};
    }
    
    // NUEVO: Normalizar este vector in-place
    Vec2& normalize() {
        double n = norm();
        if (n != 0.0) {
            x /= n;
            y /= n;
        }
        return *this;
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
    
    // NUEVO: Operador +=
    Vec2& operator+=(const Vec2& b) {
        x += b.x;
        y += b.y;
        return *this;
    }
    
    // NUEVO: Operador -=
    Vec2& operator-=(const Vec2& b) {
        x -= b.x;
        y -= b.y;
        return *this;
    }
    
    // NUEVO: Operador *=
    Vec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }
    
    // NUEVO: Operador /=
    Vec2& operator/=(double s) {
        x /= s;
        y /= s;
        return *this;
    }
    
    // NUEVO: Producto cruzado 2D (devuelve la magnitud en z)
    double cross(const Vec2& b) const {
        return x * b.y - y * b.x;
    }
    
    // NUEVO: Ángulo con respecto al eje X positivo (en radianes)
    double angle() const {
        return std::atan2(y, x);
    }
    
    // NUEVO: Rotar vector (ángulo en radianes)
    Vec2 rotated(double theta) const {
        double ct = std::cos(theta);
        double st = std::sin(theta);
        return {x * ct - y * st, x * st + y * ct};
    }
    
    Vec2 operator-() const {
    return Vec2(-x, -y);
    }
};

// Funciones inline de ayuda (ya existentes)
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

// NUEVO: Producto cruzado 2D (versión función)
inline double cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

inline double norm(const Vec2& v) {
    return std::sqrt(v.x*v.x + v.y*v.y);
}

// NUEVO: Vector normalizado (versión función)
inline Vec2 normalized(const Vec2& v) {
    double n = norm(v);
    if (n == 0.0) return {0.0, 0.0};
    return {v.x / n, v.y / n};
}

inline double distance(const Vec2& a, const Vec2& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// NUEVO: Ángulo entre dos vectores (en radianes)
inline double angle_between(const Vec2& a, const Vec2& b) {
    double n = norm(a) * norm(b);
    if (n == 0.0) return 0.0;
    
    double cos_theta = dot(a, b) / n;
    // Clamp para evitar errores numéricos
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta);
}

// NUEVO: Convertir grados a radianes
inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// NUEVO: Convertir radianes a grados
inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

} // namespace mesh_adapt
