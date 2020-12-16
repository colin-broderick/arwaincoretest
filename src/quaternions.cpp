/* Copyright (c) 2012 Alex Allen, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "quaternions.h"
#include <math.h>
 
quaternion::quaternion(double real, double im_i, double im_j, double im_k)
{
    a = real;
    b = im_i;
    c = im_j;
    d = im_k;
}
 
quaternion quaternion::operator+(quaternion quat2)
{
    quaternion temp;
    
    temp.a = a + quat2.a;
    temp.b = b + quat2.b;
    temp.c = c + quat2.c;
    temp.d = d + quat2.d;
    
    return temp;
}
 
quaternion quaternion::operator-(quaternion quat2)
{
    quaternion temp;
    
    temp.a = a - quat2.a;
    temp.b = b - quat2.b;
    temp.c = c - quat2.c;
    temp.d = d - quat2.d;
    
    return temp;
}
 
quaternion quaternion::operator*(quaternion quat2)
{
    quaternion temp;
    double e=quat2.a, f=quat2.b, g=quat2.c, h=quat2.d;
    
    temp.a = a*e -b*f -c*g -d*h;
    temp.b = a*f +b*e +c*h -d*g;
    temp.c = a*g -b*h +c*e +d*f;
    temp.d = a*h +b*g -c*f +d*e;
    
    return temp;
}
 
 
quaternion quaternion::operator*(double num)
{
    quaternion temp;
    
    temp.a = num*a;
    temp.b = num*b;
    temp.c = num*c;
    temp.d = num*d;
    
    return temp;
}
 
quaternion quaternion::operator/(double num)
{
    quaternion temp;
    
    temp.a = a/num;
    temp.b = b/num;
    temp.c = c/num;
    temp.d = d/num;
    
    return temp;
}
 
quaternion quaternion::operator+=(quaternion quat2)
{
    quaternion temp;
    
    temp.a = a + quat2.a;
    temp.b = b + quat2.b;
    temp.c = c + quat2.c;
    temp.d = d + quat2.d;
    
    return temp;
}
 
quaternion quaternion::operator-=(quaternion quat2)
{
    quaternion temp;
    
    temp.a = a - quat2.a;
    temp.b = b - quat2.b;
    temp.c = c - quat2.c;
    temp.d = d - quat2.d;
    
    return temp;
}
 
quaternion quaternion::operator*=(quaternion quat2)
{
    quaternion temp;
    double e=quat2.a, f=quat2.b, g=quat2.c, h=quat2.d;
    
    temp.a = a*e -b*f -c*g -d*h;
    temp.b = a*f +b*e +c*h -d*g;
    temp.c = a*g -b*h +c*e +d*f;
    temp.d = a*h +b*g -c*f +d*e;
    
    return temp;
}
 
quaternion quaternion::operator*=(double num)
{
    quaternion temp;
    
    temp.a = num*a;
    temp.b = num*b;
    temp.c = num*c;
    temp.d = num*d;
    
    return temp;
}
 
quaternion quaternion::operator/=(double num)
{
    quaternion temp;
    
    temp.a = a/num;
    temp.b = b/num;
    temp.c = c/num;
    temp.d = d/num;
    
    return temp;
}
      
double quaternion::mag()
{
    return sqrt(a*a + b*b + c*c + d*d);
}
 
quaternion quaternion::unit()
{
    quaternion temp;
    double size = mag();
    
    temp.a = a/size;
    temp.b = b/size;
    temp.c = c/size;
    temp.d = d/size;
    
    return temp;
}
 
quaternion quaternion::conj()
{
    quaternion temp;
    
    temp.a = a;
    temp.b = -b;
    temp.c = -c;
    temp.d = -d;
    
    return temp;
}
 
quaternion quaternion::inv()
{
    // 1/quat = conj / (quat*conj), but quat*conj = a^2 + b^2 + ... 
    return conj()/(a*a + b*b + c*c + d*d);
    
}
 
quaternion quaternion::unit_inv()
{
    return this->conj();
}
 
double quaternion::getRe() 
{
    return a;
}
double quaternion::getIm_i() 
{
    return b;
}
 
double quaternion::getIm_j()
{
    return c;
}
 
double quaternion::getIm_k()
{ 
    return d;
}
