/*
 * Path classes to assist in path planning.
 *
 * This file defines all the path classes used internally by the path 
 * planning algorithms.
 * 
 * NOTE: file contains c++11 syntax
 * 
 * Andres Rodriguez Reina
 * 2016.05.05
*/

#include <math.h>
#include <vector>
#include <algorithm>
#include <memory>

#ifdef ENABLE_PATH_GRAPHICS
    #include <SFML/Graphics.hpp>
#endif

//using namespace std;

template<class FLOATING> FLOATING inline pow2(FLOATING v)
    {return(v*v);}
template<class FLOATING> bool inline infinitesimo(FLOATING val, FLOATING ref)
    {return(abs(val*1e12)<abs(ref));}

template<class FLOATING> struct R2VECT
{
    union{
        struct{
            FLOATING x;
            FLOATING y;
        };
        FLOATING c[2];
    };
    FLOATING module2() const
        {return x*x+y*y;}
    FLOATING module() const
        {return sqrt(x*x+y*y);}
    R2VECT<FLOATING> unit() const
        {return *this/module();}
    R2VECT<FLOATING>(FLOATING mx, FLOATING my)
        {x=mx; y=my;}
    R2VECT<FLOATING>()
        {x=0.0; y=0.0;}
    FLOATING &operator[] (int i)
        {return c[i];}
    R2VECT<FLOATING>  operator+ (const R2VECT<FLOATING> &v)const 
        {return R2VECT(x + v.x,y + v.y);}
    R2VECT<FLOATING>  operator- (const R2VECT<FLOATING> &v)const 
        {return R2VECT(x - v.x,y - v.y);}
    R2VECT<FLOATING>  operator* (const FLOATING v)const 
        {return R2VECT(x * v,y * v);}
    friend R2VECT<FLOATING>  operator* (const FLOATING v,const R2VECT<FLOATING> &u)
        {return R2VECT(u.x * v,u.y * v);}
    R2VECT<FLOATING>  operator/ (const FLOATING v)const 
        {return R2VECT(x / v,y / v);}
    FLOATING  operator^ (const R2VECT<FLOATING> &v) const //cross product
        {return -y*v.x+x*v.y;}
    FLOATING  operator* (const R2VECT<FLOATING> &v) const //dot product
        {return x*v.x+y*v.y;}
    R2VECT<FLOATING> &operator*=(const FLOATING v)
        {*this=*this*v; return *this; }
    R2VECT<FLOATING> &operator/=(const FLOATING v)
        {*this=*this/v; return *this; }
    R2VECT<FLOATING> &operator+=(const R2VECT<FLOATING> &v)
        {*this=*this+v; return *this; }
    R2VECT<FLOATING> &operator-=(const R2VECT<FLOATING> &v)
        {*this=*this-v; return *this; }
    bool  operator== (const R2VECT<FLOATING> &v) const
        {return infinitesimo(x-v.x,x+v.x)&&infinitesimo(y-v.y,y+v.y);}
    R2VECT<FLOATING> module(FLOATING m)
        {return unit()*m;}
    R2VECT<FLOATING>  operator+ ()const 
        {return *this;}
    R2VECT<FLOATING>  operator-()const 
        {return *this*((FLOATING)-1.0);}
};

typedef R2VECT<double> vect2;

vect2 vect2angle(double a)
{
    return vect2(cos(a),sin(a));
}

double fmod2pi(double v)
{
    return fmod(fmod(v,M_PI*2)+M_PI*4,M_PI*2);
}

struct pose2d
{
    vect2 p; //position
    double t; //angle
    pose2d(double x,double y,double theta):p(x,y),t(theta){}
    pose2d(vect2 pos,double theta):p(pos),t(theta){}
    pose2d(){};
#ifdef ENABLE_PATH_GRAPHICS
    void draw(sf::RenderWindow& a, sf::Color c=sf::Color(0, 0, 255),int rad = 40) const
    {
        vect2 p1=p+vect2angle(t)*rad*0.8;
        vect2 p2=p+vect2angle(t)*rad;
        vect2 p3=p1+vect2angle(t+M_PI/2)*rad*.1/cos(M_PI/6);
        vect2 p4=p1-vect2angle(t+M_PI/2)*rad*.1/cos(M_PI/6);
        sf::Vertex vertices[8] =
        {
            sf::Vertex(sf::Vector2f(p.x,p.y), c),
            sf::Vertex(sf::Vector2f(p1.x,p1.y), c),
            sf::Vertex(sf::Vector2f(p2.x,p2.y), c),
            sf::Vertex(sf::Vector2f(p3.x,p3.y), c),
            sf::Vertex(sf::Vector2f(p3.x,p3.y), c),
            sf::Vertex(sf::Vector2f(p4.x,p4.y), c),
            sf::Vertex(sf::Vector2f(p4.x,p4.y), c),
            sf::Vertex(sf::Vector2f(p2.x,p2.y), c)
        };
        a.draw(vertices, 8, sf::Lines);
    }
#endif
};

class path_segment
{
protected:
    pose2d start;
    pose2d end;
    double length;
public:
    double curve_start;
    virtual void get_position(double curve_length, pose2d & position)=0;
    virtual path_segment* clone() const = 0;
    bool  operator< (const path_segment & v) const
        {return curve_start<v.curve_start;}
    bool  operator< (double v) const
        {return curve_start<v;}
    pose2d get_start(){return start;}
    pose2d get_end(){return end;}
    double get_length(){return length;}
};

template <class Base, class Derived>
class make_clonable : public Base
{
public:
    Base* clone() const {
        return new Derived(static_cast<const Derived&>(*this));
    }
};

class path_line: public make_clonable<path_segment,path_line>
{
public:
    vect2 unit;
    path_line(pose2d _start,pose2d _end)
    {
        start=_start;
        end=_end;
        vect2 vec=end.p-start.p;
        length=vec.module();
        unit=vec/length;
    }
    void get_position(double curve_length, pose2d & position)
    {
        position=pose2d(start.p+unit*curve_length,start.t);
    }
};

class path_arc: public make_clonable<path_segment,path_arc>
{
public:
    vect2 center;
    double radious;
    vect2 cx,cy;
    path_arc(pose2d _start,double _radious, double _length):radious(_radious)
    //NOTE: positive radious turns clock-wise, negative counter-clock-wise
    {
        start=_start;
        vect2 tmp = vect2angle(start.t);
        cy=radious*tmp;
        cx=-radious*vect2(-tmp.y,tmp.x);
        center=start.p-cx;
        length=_length;
        get_position(length,end);
    }
    void get_position(double curve_length, pose2d & position)
    {
        double th=curve_length/radious;
        position=pose2d(center+cx*cos(th)+cy*sin(th),fmod2pi(th+start.t));
    }
};

//ACTUAL PATH CLASS
class path
{
private:
    typedef std::unique_ptr<path_segment> p_path_segment;
    typedef std::vector<p_path_segment>::iterator list_iterator;
    typedef std::vector<p_path_segment>::const_iterator const_list_iterator;
    std::vector<p_path_segment> d;
    double min_radious;
    double length;
    void shortest_path_between(pose2d start,pose2d end)
    {
        end.t=fmod2pi(end.t);
        start.t=fmod2pi(start.t);
        vect2 t0[4];
        vect2 t1[4];
        double at0[4];
        double at1[4];
        double al0[4];
        double al1[4];
        double dist[4];
        int imin=0;
        {
            double dmin=1e150;
            for(int i=0;i<4;i++)
            {
                int s0=(i%2)*2-1;
                int s1=(i/2)*2-1;
                vect2 c0 = vect2angle(start.t+(M_PI/2)*s0)*min_radious+start.p;
                vect2 c1 = vect2angle(end.t+(M_PI/2)*s1)*min_radious+end.p;
                vect2 d = c1-c0;
                double dmod=d.module();
                double alpha=(s0!=s1)?asin(2*min_radious/dmod):0.;
                double ad=atan2(d.y,d.x);
                double ap0=start.t-M_PI/2*s0;
                double ap1=end.t-M_PI/2*s1;
                at0[i]=dmod<0.0000001?ap0:ad+(alpha-M_PI/2)*s0;
                at1[i]=dmod<0.0000001?ap0:ad+(-alpha-M_PI/2)*s1;
                al0[i]=fmod2pi(-s0*(ap0-at0[i]));
                al1[i]=fmod2pi(-s1*(at1[i]-ap1));
                al0[i]=(std::min(2*M_PI-al0[i],al0[i])<0.0000001/min_radious)?0.0:al0[i];
                al1[i]=(std::min(2*M_PI-al1[i],al1[i])<0.0000001/min_radious)?0.0:al1[i];
                t0[i]=c0+vect2angle(at0[i])*min_radious;
                t1[i]=c1+vect2angle(at1[i])*min_radious;
                dist[i]=(t0[i]-t1[i]).module()+(al0[i]+al1[i])*min_radious;
                imin=dist[i]<dmin?i:imin;
                dmin=dist[i]<dmin?dist[i]:dmin;
            }
        }
        {
            int s0=(imin%2)*2-1;
            int s1=(imin/2)*2-1;
            pose2d pt0(t0[imin],fmod2pi(at0[imin]+M_PI/2*s0));
            pose2d pt1(t1[imin],fmod2pi(at1[imin]+M_PI/2*s1));
            if(al0[imin]!=0.0)
            {
                d.push_back(p_path_segment(
                    new path_arc(start,s0*min_radious,al0[imin]*min_radious)));
                d.back()->curve_start=length;
                length+=al0[imin]*min_radious;
            }
            if((t0[imin]-t1[imin]).module()>0.0000001)
            {
                d.push_back(p_path_segment(
                    new path_line(pt0,pt1)));
                d.back()->curve_start=length;
                length+=(t0[imin]-t1[imin]).module();
            }
            if(al1[imin]!=0.0)
            {
                d.push_back(p_path_segment(
                    new path_arc(pt1,s1*min_radious,al1[imin]*min_radious)));
                d.back()->curve_start=length;
                length+=al1[imin]*min_radious;
            }
        }
        {
            if(!(abs(dist[imin]-length)<1e-15))
                std::cout<<"error assert\n";
            if(!((length<0.00001) && ((start.p-end.p).module()<0.0001)))
            {
                pose2d endp,startp;
                get_position(length,endp);
                get_position(0.0,startp);
                if(std::abs(endp.p.x-end.p.x)+std::abs(endp.p.y-end.p.y)+std::abs(endp.t-end.t)>0.0001)
                    {std::cout<<"ERROR calculated curve end doesnt't match\n";while(1);}
                if(std::abs(start.p.x-startp.p.x)+std::abs(start.p.y-startp.p.y)+std::abs(start.t-startp.t)>0.0001)
                    {std::cout<<"ERROR calculated curve start doesnt't match\n";while(1);}
            }
        }
    }
public:
    path(double min_radious_):min_radious(min_radious_),length(0){};
    path():path(0){};
    path(pose2d start,pose2d end,double min_radious_):min_radious(min_radious_),length(0)
    {
        shortest_path_between(start, end);
    }
    path(const path & p):min_radious(p.min_radious),length(p.length)
    {
        for(const_list_iterator it=p.d.begin();it<p.d.end();it++)
            d.push_back(p_path_segment((*it)->clone()));
    }
    friend void swap(path & p, path & q)
    {
        std::swap(q.min_radious,p.min_radious);
        std::swap(q.length,p.length);
        std::swap(q.d,p.d);
    }
    path(path && p):path()
    {
        swap(*this,p);
    }
    path& operator=(path other)
    {
        swap(*this, other);
        return *this;
    }
    pose2d get_start(){return d.front()->get_start();}
    pose2d get_end(){return d.back()->get_end();}
    bool get_position(double curve_length, pose2d & position) const
    {
        if(d.size()==0)
            return false;
        if(curve_length==0)
        {
            d.front()->get_position(curve_length-d.front()->curve_start,position);
            return true;
        }
        if(curve_length>length)
            return false;
        auto it = std::upper_bound(d.rbegin(), d.rend(), curve_length,[](double b, const p_path_segment &a)->bool{return ( (*a)<b);});
        if(it==d.rend())
            return false;
        (*it)->get_position(curve_length-(*it)->curve_start,position);
        return true;
    }
    pose2d get_position_rel(double curve_length) const
    {
        pose2d ret;
        curve_length=std::min(1.0,std::max(0.0,curve_length));
        get_position(curve_length/length,ret);
        return ret;
    }
    double get_length() const
    {
        return length;
    }
    bool append(path p)
    {
        int s = d.size();
        pose2d endp,startp;
        if(get_position(length,endp))
            if(p.get_position(0.0,startp))
                if(std::abs(endp.p.x-startp.p.x)+std::abs(endp.p.y-startp.p.y)+std::abs(endp.t-startp.t)>0.0001)
                    {std::cout<<"ERROR curve end and appended begin don't match\n";while(1);}
        std::move(p.d.begin(), p.d.end(), std::back_inserter(d));
        for(int i=s;i<d.size();i++)
        {
            d[i]->curve_start=length;
            length+=d[i]->get_length();
        }
    }
    bool shortcut(double curve_length_start, double curve_length_end)
    {
        double old_length=length;
        std::vector<p_path_segment> temp;
        std::swap(d,temp);  //swap the content with temporary (no-cost)
        auto it=temp.begin();
        for(;it<temp.end();it++) //iterate to find segment to break (beginning)
        {
            if((*it)->curve_start+(*it)->get_length()<curve_length_start)
                d.push_back(std::move(*it));
            else
                break;
        }
        if(it==temp.end()) //not found, return false
            return false;
        pose2d  new_start,  //beginning of segment to break (beginning)
                new_pre_start;  //break point in segment
        (*it)->get_position(0.0,new_pre_start);
        (*it)->get_position(curve_length_start-(*it)->curve_start,new_start);
//        new_start.draw(*gwin);
//        gwin->display();
//        new_pre_start.draw(*gwin);
//        gwin->display();
        if(d.size()>0)
            length=d.back()->curve_start+d.back()->get_length();
        else
            length=0.0;
        if((*it)->curve_start!=length){std::cout<<"ERROR curve_start_pos doesn't match\n";while(1);}
        append(path(new_pre_start,new_start,min_radious));  //append part of the broken segment we want to keep (beginning)
        for(;it<temp.end();it++)    //iterate to find segment to break (end)
        {
            if(!((*it)->curve_start+(*it)->get_length()<curve_length_end))
                break;
        }
        if(it==temp.end()) //not found. just append path to end ¿?  (end)
        {
            pose2d true_end;
            temp.back()->get_position(old_length,true_end);
            append(path(new_start,true_end,min_radious));
            std::cout<<"ERROR curve_end_pos too far\n";
            return false;
        }
        pose2d  new_end,  // breaking point (end)
                new_post_end; //end of the broken segment (end)
        (*it)->get_position(curve_length_end-(*it)->curve_start,new_end);
        (*it)->get_position((*it)->get_length(),new_post_end);
//        new_end.draw(*gwin);
//        gwin->display();
//        new_post_end.draw(*gwin);
//        gwin->display();
        append(path(new_start,new_end,min_radious));  //add new path between break points
        append(path(new_end,new_post_end,min_radious));  //add path between break point and end of segment
        it++; //skip the broken segment
        for(;it<temp.end();it++)  //append all remaining segments
        {
            d.push_back(std::move(*it));
            d.back()->curve_start=length;
            length+=d.back()->get_length();
        }
        return true;
    }
#ifdef ENABLE_PATH_GRAPHICS
    void draw(sf::RenderWindow& a, sf::Color c_start=sf::Color(255, 0, 0), sf::Color c_path=sf::Color(0, 0, 255), sf::Color c_end=sf::Color(0, 255, 0),int step = 30) const
    {
        pose2d position;
        for(double i = step;;i+=step)
        {
            if(!get_position(i, position))break;
            position.draw(a, c_path,4*step/3);
        }
        if(get_position(0, position))
            position.draw(a, c_start,4*step/3);
        if(get_position(length, position))
            position.draw(a, c_end,4*step/3);
    }
#endif
};





