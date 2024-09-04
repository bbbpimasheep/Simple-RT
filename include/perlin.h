#pragma once
#ifndef PERLIN_H
#define PERLIN_H

#include "global.h"
#include "mathematics.h"

class Perlin {
public:
    // Constructor
    Perlin() {
        for (size_t i = 0; i < point_count; i += 1)
            rand_vectors[i] = RandomVec3Unit();
        
        GeneratePerlin(perlin_x);
        GeneratePerlin(perlin_y);
        GeneratePerlin(perlin_z);
    }
    
    // Methods
    double Noise(const Point3& point) const {
        auto u = point.x - Floor(point.x);
        auto v = point.y - Floor(point.y);
        auto w = point.z - Floor(point.z);

        int i = Floor(point.x);
        int j = Floor(point.y);
        int k = Floor(point.z);
        Vector3 cube[2][2][2];
        for (int di=0; di<2; di+=1) 
            for (int dj=0; dj<2; dj+=1) 
                for (int dk=0; dk<2; dk+=1) 
                    cube[di][dj][dk] = rand_vectors[
                        perlin_x[(i+di) & 255] ^ 
                        perlin_y[(j+dj) & 255] ^ 
                        perlin_z[(k+dk) & 255]];

        return PerlinInterpolate(cube, u, v, w);
    }
    double Turbulence(const Point3& point, int depth=7) const {
        auto accum = 0.0;
        auto temp_point = point;
        double weight = 1.0;
        for (int i=0; i<depth; i+=1) {
            accum += weight * Noise(temp_point);
            weight *= 0.5;
            temp_point *= 2;
        }
        return Abs(accum);
    }

private:
    // Members
    static const int point_count = 256;
    Vector3 rand_vectors[point_count];
    int perlin_x[point_count];
    int perlin_y[point_count];
    int perlin_z[point_count];

    // Methods
    void GeneratePerlin(int *perlin) {
        for (size_t i = 0; i < point_count; i += 1)
            perlin[i] = i;
        for (size_t i = point_count-1; i > 0; i -= 1) {
            int shift = RandomInt(0, i);
            int temp  = perlin[i];
            perlin[i] = perlin[shift];
            perlin[shift] = temp;
        }
    }
    static double PerlinInterpolate(const Vector3 cube[2][2][2], double u, double v, double w) {
        auto uu = u * u * (3-2*u);
        auto vv = v * v * (3-2*v);
        auto ww = w * w * (3-2*w);
        auto accum = 0.0;

        for (int i=0; i<2; i+=1)
            for (int j=0; j<2; j+=1)
                for (int k=0; k<2; k+=1) {
                    Vector3 weight(u-i, v-j, w-k);
                    accum += (i*uu + (1-i)*(1-uu)) *
                             (j*vv + (1-j)*(1-vv)) *
                             (k*ww + (1-k)*(1-ww)) *
                             Dot(cube[i][j][k], weight);
                }
        return accum;
    }
};

#endif // PERLIN_H