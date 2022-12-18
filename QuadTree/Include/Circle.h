#ifndef CIRCLE_H
#define CIRCLE_H

namespace Recast
{
    class Circle
    {
    public:
        Circle()
        :mX(0)
        ,mY(0)
        ,mRadius(0)
        {}
        
        Circle(float x, float y, float radius)
        :mX(x)
        ,mY(y)
        ,mRadius(radius)
        {}
        
        inline float GetX() const
        {
            return mX;
        }
        
        inline float GetY() const
        {
            return mY;
        }
        
        inline void SetXY(float x, float y)
        {
            mX = x;
            mY = y;
        }
        
        inline float GetRadius() const
        {
            return mRadius;
        }
        
        inline void SetRadius(float radius)
        {
            mRadius = radius;
        }
        
        inline float DistanceSqr(float x, float y) const
        {
            float xDiff = mX - x;
            float yDiff = mY - y;
            return xDiff * xDiff + yDiff * yDiff;
        }
        
        inline bool IsContain(float x, float y) const
        {
            float distSqr = DistanceSqr(x, y);
            float radiusSqr = mRadius * mRadius;
            return distSqr <= radiusSqr;
        }
        
        inline bool Intersect(const Circle& other) const
        {
            float distSqr = DistanceSqr(other.mX, other.mY);
            float len = mRadius + other.mRadius;
            float lenSqr = len * len;
            return distSqr <= lenSqr;
        }
        
    private:
        float mX;
        float mY;
        float mRadius;
    };
}

#endif // CIRCLE_H
