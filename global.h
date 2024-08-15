#pragma once
#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <limits>
#include <memory>
#include <iomanip>
#include <chrono>

using std::shared_ptr;
using std::make_shared;
using std::fixed;
using std::setprecision;

constexpr double EPS_UNIT = 1e-4;
constexpr double EPS_DEUX = 1e-8;
constexpr double EPS_QUAT = 1e-16;
constexpr double GAMMA = 1.0 /2.2;
constexpr double POS_INF = std::numeric_limits<double>::infinity();
constexpr double NEG_INF = -POS_INF;

template <typename T> 
inline std::string Str(T value) { return std::to_string(value); }
template <typename T>
inline T Sqr(T value) { return value*value; }
template <typename T>
inline double Sqrt(T value) { return std::sqrt(value); }
template <typename T>
inline T Abs(T value) { return std::abs(value); }
template <typename T>
inline void Swap(T& value1, T& value2) { std::swap(value1, value2); }
template <typename T>
inline double Sin(T value) { return std::sin(value); }
template <typename T>
inline double Cos(T value) { return std::cos(value); }
template <typename T>
inline double Tan(T value) { return std::tan(value); }
template <typename T>
inline T Min(T value1, T value2) { return std::min(value1, value2); }
template <typename T>
inline T Max(T value1, T value2) { return std::max(value1, value2); }

inline double RandomFloat(){
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::uniform_real_distribution<double> dist(0.f, 1.f); // distribution in range [0, 1]
    return dist(rng);
}
inline double RandomFloat(double a, double b) { return a + (b - a) * RandomFloat(); }
inline double DegtoRad(double degrees) { return degrees * M_PI / 180.0; }

inline void ProgressBar(double progress) {
    int width = 80;
    int ratio = int(progress * width);
    std::clog << "\r[";
    for (int i = 0; i < width; i += 1) {
        if (i < ratio) std::clog << '#';
        else std::clog << '=';
    }
    std::clog << "] " << fixed << setprecision(1) << progress*100.0 << "%" << std::flush;
}

struct Intersection;
class  Vector3;
class  Ray;
class  Sphere;
class  Interval;
class  Material;
class  Bounds3;


#endif // GLOBAL_H