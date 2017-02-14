/*
 * RRT implementation for path planning.
 *
 * This file defines all the path classes used internally by the path 
 * planning algorithms.
 * 
 * NOTE: file contains c++11 syntax
 * 
 * Andres Rodriguez Reina
 * 2016.05.05
*/

#include "path.hpp"

double rand_between(double x,double y)
{
    return rand()*(y-x)/RAND_MAX+x;
}

bool check_between(double v, double mx,double mn)
{
    return (v<mx) && (v>mn); 
}

double interpolate(double x, double y, double a)
{
    return (y-x)*a+x;
}

typedef bool (*collision_checker_f_prototype)(double x, double y, double t);

template<typename Tf>
struct path_planner_functions
{
    typedef pose2d Q;
    typedef path E;
    typedef path P;
    double min_radious=30;
    Tf collision_checker_f;
    path_planner_functions(double min_radious_, Tf collision_checker_f_):min_radious(min_radious_),collision_checker_f(collision_checker_f_){}
    Q sample()
    {
        return Q(rand()%600+200,rand()%500+200,rand());
    }
    Q interpolate(const E &x, double a)
    {
        return x.get_position_rel(a);
    }
    E make_edge(const Q &x,const Q &y)
    {
        return E(x,y,min_radious);
    }
    bool valid(const Q &x)
    {
        return collision_checker_f(x.p.x,x.p.y,x.t);
    }
    bool valid(const E &x)
    {
        pose2d position;
        for(double i = 0.;;i+=5.)
        {
            if(!x.get_position(i, position))return true;
            if(!valid(position))
                return false;
        }
    }
    double distance(const Q &x,const Q &y)
    {
        return sqrt(pow2(x.p.x-y.p.x)+pow2(x.p.y-y.p.y));
    }
    double distance(const E &x)
    {
        return x.get_length();
    }
    P initP()
    {
        return P(min_radious);
    }
    void append_edge(P & p, E e)
    {
        p.append(e);
    }
    void make_shortcut(P & p,double p1, double p2)
    {
        p.shortcut(p1,p2);
    }
};

template<typename F>
typename F::P RRT(F world, typename F::Q start, typename F::Q goal, int n=100000, bool verbose=false, typename std::vector<typename F::E>* treeout=0)
{
    if(!world.valid(goal))
    {
        if(verbose)std::cout<<"error! GOAL IS INVALID!!\n";
        return typename F::P();
    }
    {
        typename F::E e=world.make_edge(start,goal);
        if(world.valid(e))
        {
            if(verbose)std::cout<<" GOAL REACHED --------------------------------------------------\n";
            typename F::P ret = world.initP();
            world.append_edge(ret,e);
            return ret;
        }
    }
    struct G
    {
        typename F::Q q;
        typename F::E e;
        int p;
    };
    std::vector<G> V;
    V.push_back(G{start,typename F::E(),-1});
    bool good=false;
    for (int i=0;i<n;i++)
    {
        if(verbose)std::cout<<"--ITERATION "<<i<<" V.size="<<V.size()<<"\n";
        typename F::Q q;
        q=world.sample();
        if(!world.valid(q))
        {
        	if(verbose)std::cout<<"   invalid sample\n";
            continue;
        }
        G* p=0;
        double mind=1.e80;
        int minj=-1;
        int j=0;
        for(auto it=V.begin();it!=V.end();it++,j++)
        {
            double d=world.distance(q,it->q);
            if(d<mind)
            {
                mind=d;
                p=&(*it);
                minj=j;
            }
        }
        if(verbose)std::cout<<" check path to closest node: "<<minj<<"\n";//<<p->d[0]<<" "<<p->d[1]<<" "<<p->d[2]<<" distance="<<mind<<"\n";
        {
            typename F::E e=world.make_edge(p->q,q);
            if(!world.valid(e))
                continue;
            if(verbose)std::cout<<" NODE ADDED ----------------------------------------------------\n";
            V.push_back(G{q,std::move(e),minj});
        }
        {
            typename F::E e=world.make_edge(q,goal);
            if(!world.valid(e))
                continue;
            if(verbose)std::cout<<" GOAL REACHED --------------------------------------------------\n";
            V.push_back(G{goal,std::move(e),(int)V.size()-1});
        }
        good=true;
        break;
    }
    if(treeout)
        for(auto i=V.rbegin();i<V.rend();i++)
            treeout->push_back(i->e);
    if(!good)
    {
        if(verbose)std::cout<<"error! PATH NOT FOUND at end of iterations!!\n";
        return typename F::P();
    }
    if(verbose)std::cout<<"Graph has "<<V.size()<<" nodes\n";
    {
        std::vector<G*> W;
        int k=V.size()-1;
        int m=V[k].p;
        while(m>=0)
        {
            if(verbose)std::cout<<"m="<<m<<" k="<<k<<"\n";
            W.push_back(&(V[k]));
            k=m;
            m=V[k].p;
        }
        if(verbose)std::cout<<"Path is "<<W.size()<<" elements long\n";
        typename F::P ret = world.initP();
        for(auto i=W.rbegin();i<W.rend();i++)
            world.append_edge(ret,std::move((*i)->e));
        if(verbose)std::cout<<"SMOOTHING\n";
        for(int i=0;i<n;i++)
        {
            double p1 = rand()*ret.get_length()/RAND_MAX;
            double p2 = rand()*ret.get_length()/RAND_MAX;
            if(p2<p1)
                std::swap(p1,p2);
            typename F::P shct=ret;//world.make_edge(world.interpolate(ret, p1),world.interpolate(ret, p2));
            world.make_shortcut(shct,p1,p2);
            if(shct.get_length()<ret.get_length()-0.001)
                if(world.valid(shct))
                    std::swap(ret,shct);
        }
        if(!world.valid(ret))
            {std::cout<<"error! PATH NOT valid after smoothing!!\n";}//while(1);}
        if(verbose)std::cout<<"**** FINISHED *************************\n";
        return ret;
    }
}
