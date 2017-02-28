/**THIS FILE CONTAINS A 2D MAP CLASS THAT CAN BE EASILY VIEWED BY HUMANS**/
//written by Leon Frickensmith and Max Archer
//leonfrickensmith@gmail.com

#ifndef MAP_HPP
#define MAP_HPP

#include <iostream>//cout
#include <fstream>//filestream
#include <stdlib.h>//abs

#include <memory.h>

#if 1
    #define ASSERT(x) if(!(x)) {std::cout<<"Error Assertion " #x "\n";*((int*)(0))=0;}
#else
    #define ASSERT(x) 
#endif


const float map_defaultValue = -9999.0f;//default value of map pieces, DO NOT REFERENCE THIS
const float map_occupied = 1;
const float map_unoccupied = 0;
const float map_unknown = map_defaultValue;



template<typename T> struct matrix_tag
{
public:
    matrix_tag(matrix_tag & m)
    {
        d=0;
        nx=0;ny=0;
        *this=m;
    }
    matrix_tag()
    {
        d=0;
        nx=0;ny=0;
    }
    int dx,dy;
    matrix_tag(int width, int height)
    {
        d=0;
        nx=0;ny=0;
        create(width,height);
    }
private:
    int nx,ny;
    T* d;
    T* g;
public:
    void create(int ax,int bx,int ay,int by)
    {
        int oldsize=nx*ny;
        dx=-ax;dy=-ay;
        nx=bx-ax+1;ny=by-ay+1;
        ASSERT(nx>0);
        ASSERT(ny>0);
        int newsize=nx*ny;
        if(d && oldsize!=newsize)
        {
            delete[] d;
            d=0;
        }
        if(!d)
            d=new T [newsize];
        g=d+dx+nx*dy;
    }
    void create(int width, int height)
    {
        create(0,width-1,0,height-1);
    }
    matrix_tag(int ax,int bx,int ay,int by)
    {
        d=0;
        nx=0;ny=0;
        create(ax,bx,ay,by);
    }
    int xllim()const
    {
        return -dx;
    }
    int xhlim()const
    {
        return -dx+nx;
    }
    int yllim()const
    {
        return -dy;
    }
    int yhlim()const
    {
        return -dy+ny;
    }
    bool validIndex(int ax,int ay)
    {
        int kx=ax+dx;int ky=ay+dy;
        return((kx<nx)
        && (ky<ny)
        && (kx>=0)
        && (ky>=0));
	}
#if 1
    T & operator() (int ax,int ay)
    {
        int kx=ax+dx;int ky=ay+dy;
        ASSERT(kx<nx);
        ASSERT(ky<ny);
        ASSERT(kx>=0);
        ASSERT(ky>=0);
        return d[kx+nx*ky];
    }
#else
    T & operator() (int ax,int ay)
    {
        return g[ax+nx*ay];
    }
#endif
#if 1
    const T & operator() (int ax,int ay) const
    {
        int kx=ax+dx;int ky=ay+dy;
        ASSERT(kx<nx);
        ASSERT(ky<ny);
        ASSERT(kx>=0);
        ASSERT(ky>=0);
        return d[kx+nx*ky];
    }
#else
    const T & operator() (int ax,int ay) const
    {
        return g[ax+nx*ay];
    }
#endif
    matrix_tag & operator= (const matrix_tag & m)
    {
        if(d && nx*ny!=m.nx*m.ny)
        {
            delete[] d;
            d=0;
        }
        if(!d)
            d=new T [m.nx*m.ny];
        nx=m.nx;
        ny=m.ny;
        dx=m.dx;
        dy=m.dy;
        g=d+dx+nx*dy;
        memcpy(d,m.d,nx * ny * sizeof(T));
        return *this;
    }
    int xSize() const
    {
        return nx;
    }
    int ySize() const
    {
        return ny;
    }
    operator void*()
    {
        return (void*)d;
    }
    T* data()
    {
        return d;
    }
    void zeroMem()
    {
        memset(d,0,nx * ny * sizeof(T));
    }
    ~matrix_tag()
    {
        if(d)
            delete[] d;
        d=0;
    }
    void fill(T val)
    {
        for(int i=0;i<nx*ny;i++)
            d[i]=val;
    }
} ;

typedef matrix_tag<float> MATRIX;
typedef matrix_tag<unsigned char> MAT_GRAYSCALE;
typedef matrix_tag<unsigned char[3]> MAT_RGB;



#endif // MAP_HPP
