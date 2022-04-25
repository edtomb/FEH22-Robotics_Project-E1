#include <cmath>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <rtxOn.h>
/**
 * @brief Does ray tracing to color a scene on the proteus. copilot wrote this entire program for me. 
 * 
 */

struct vector{
    float x,y,z;
    vector(){
        x=0;
        y=0;
        z=0;
    }
    vector(float a, float b, float c){
        x=a;
        y=b;
        z=c;
    }
    vector operator+(vector a){
        return vector(x+a.x,y+a.y,z+a.z);
    }
    vector operator-(vector a){
        return vector(x-a.x,y-a.y,z-a.z);
    }
    vector operator*(float a){
        return vector(x*a,y*a,z*a);
    }
};
struct Ray{
    
    vector origin;
    vector direction;
    Ray(vector o, vector d){
        origin=o;
        direction=d;
    }
};
class Sphere{
    public:
        vector center;
        float radius;
        Sphere(vector c, float r){
            center=c;
            radius=r;
        }
        bool intersect(Ray ray, float &t){
            //Direction unit vector of ray, 
            vector d=ray.direction;
            //Origin of ray, or light source
            vector o=ray.origin;
            //Distance between ray origin and sphere center
            vector oc=o-center;
            //The sphere intersection equation from wikipedia
            float a=d.x*d.x+d.y*d.y+d.z*d.z;
            float b=2*(oc.x*d.x+oc.y*d.y+oc.z*d.z);
            float c=oc.x*oc.x+oc.y*oc.y+oc.z*oc.z-radius*radius;
            float disc=b*b-4*a*c;
            if(disc<0)
                return false;
            float t0=(-b-sqrt(disc))/(2*a);
            float t1=(-b+sqrt(disc))/(2*a);
            if(t0<0 && t1<0)
                return false;
            if(t0<0)
                t=t1;
            else if(t1<0)
                t=t0;
            else

                t=t0<t1?t0:t1;
            return true;
        }
};


//get intersection between ray and sphere

double dot(vector a, vector b){
    return a.x*b.x+a.y*b.y+a.z*b.z;
}
void colorPixel(int x, int y, int r, int g, int b){
    //convert rgb to 16 bit color
    int color=r<<11|g<<5|b;
    LCD.SetFontColor(color);
    LCD.DrawPixel(x,y);
}
void render(){
    int w=320;
    int h=240;
    Sphere spheres[2]={Sphere(vector(10,20,10),5),Sphere(vector(30,40,20),10)};
    LCD.Clear();
    for(int y=0;y<240;y++){
        for(int x=0;x<320;x++){
            Ray ray(vector(x,y,0),vector(0,0,1));
            float t=100000;
            if(spheres[0].intersect(ray,t)){
                //find the normal vector of the sphere at the intersection point
                vector p=ray.origin+ray.direction*t;
                vector n=p-spheres[0].center;
                n.x/=spheres[0].radius;
                n.y/=spheres[0].radius;
                n.z/=spheres[0].radius;
                //color the pixel based on the normal vector
                colorPixel(x,y,255*n.x,255*n.y,255*n.z);
            }
            else if(spheres[1].intersect(ray,t)){
                vector p=ray.origin+ray.direction*t;
                vector n=p-spheres[1].center;
                n.x/=spheres[1].radius;
                n.y/=spheres[1].radius;
                n.z/=spheres[1].radius;
                colorPixel(x,y,255*n.x,255*n.y,255*n.z);
            }
            else{
                colorPixel(x,y,0,0,0);
            }
        }
    }
}
