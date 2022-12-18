#ifndef AABB_H
#define AABB_H

namespace Recast
{
    class AABB
    {
    public:
        AABB()
        :mX(0)
        ,mY(0)
        ,mWidth(0)
        ,mHeight(0)
        {}
        
        AABB(float x, float y, float width, float height)
        :mX(x)
        ,mY(y)
        ,mWidth(width)
        ,mHeight(height)
        {}
        
        AABB(const AABB& other)
        :mX(other.mX)
        ,mY(other.mY)
        ,mWidth(other.mWidth)
        ,mHeight(other.mHeight)
        {}
        
        inline void SetXY(float x, float y)
        {
            mX = x;
            mY = y;
        }
        
        inline void SetSize(float width, float height)
        {
            mWidth = width;
            mHeight = height;
        }
        
        inline float GetLeft() const
        {
            return mX;
        }
        
        inline float GetRight() const
        {
            return mX + mWidth;
        }
        
        inline float GetBottom() const
        {
            return mY;
        }
        
        inline float GetTop() const
        {
            return mY + mHeight;
        }
        
        inline float GetCenterX() const
        {
            return mX + mWidth * 0.5f;
        }
        
        inline float GetCenterY() const
        {
            return mY + mHeight * 0.5f;
        }
        
        inline float GetWidth() const
        {
            return mWidth;
        }
        
        inline float GetHeight() const
        {
            return mHeight;
        }
        
        inline bool Intersect(const AABB& other) const
        {
            return !(GetRight() < other.GetLeft() ||
                     GetLeft() > other.GetRight() ||
                     GetTop() < other.GetBottom() ||
                     GetBottom() > other.GetTop());
        }
        
        inline bool IsContain(const AABB& other) const
        {
            return !(GetLeft() > other.GetLeft() ||
                     GetRight() < other.GetRight() ||
                     GetBottom() > other.GetBottom() ||
                     GetTop() < other.GetTop());
        }
        
        inline bool IsContain(float x, float y) const
        {
            return x >= GetLeft() && x <= GetRight() &&
            y >= GetBottom() && y <= GetTop();
        }
        
    private:
        float mX;
        float mY;
        float mWidth;
        float mHeight;
    };
}

#endif // AABB_H
