#ifndef MAP_TRANSFORM_HPP
#define MAP_TRANSFORM_HPP
#include "Linear.hpp"

struct MapTransformer {
    const float PI = 3.14159265;
    Vec2f rotate_point(Vec2f p, float angle)
    {
        angle = angle * PI / 180.0;
        float s = sin(angle);
        float c = cos(angle);

        // rotate point
        float xNew = p.x * c - p.y * s;
        float yNew = p.x * s + p.y * c;

        p.x = xNew;
        p.y = yNew;

        return p;
    }
    Vec2f translate_point(Vec2f p, int x, int y)
    {
        p.x += x;
        p.y += y;
        return p;
    }
};
#endif