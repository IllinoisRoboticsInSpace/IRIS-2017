/*
 * MapTransformer.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: Rohit Singh
 */

#include "Map.hpp"
#include "Linear.hpp"
#include <iostream>
using namespace std;

const float PI = 3.14159265;
const int gradientHalfSizeX = 80;
const int gradientHalfSizeY = 80;
const int historicHalfSizeX = 80;
const int historicSizeY = 180;


Vec2f rotate_point(Vec2f p, float angle){
    angle = angle*PI/180.0;
    float s = sin(angle);
    float c = cos(angle);

    // rotate point
    float xNew = p.x * c - p.y * s;
    float yNew = p.x * s + p.y * c;

    p.x = xNew;
    p.y = yNew;

    return p;
}
Vec2f translate_point(Vec2f p, int x, int y){
    p.x += x;
    p.y += y;
    return p;
}

void transformMap(MATRIX local, MATRIX historic, double theta, int x, int y){
    for(int i = -gradientHalfSizeX; i<gradientHalfSizeX; i++){
        for(int j = -gradientHalfSizeY; j<gradientHalfSizeY; j++){
            if (local(i,j)==map_occupied || local(i,j)==map_unoccupied){
                Vec2f p = Vec2f(i,j);
                p = rotate_point(p,theta);
                p = translate_point(p,x,y);
                historic(int(p.x), int(p.y)) = local(i,j);
            }
        }
    }
}


int main(){
    Vec2f f = Vec2f(1, 0);
    f = rotate_point(f,90);
    f = translate_point(f,1,1);
    cout << f.x << "  " << f.y;
}