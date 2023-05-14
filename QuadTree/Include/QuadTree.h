#ifndef QUAD_TREE_H
#define QUAD_TREE_H

#include <vector>
#include "AABB.h"
#include "Circle.h"

#define RIGHT_TOP           0
#define LEFT_TOP            1
#define LEFT_BOTTOM         2
#define RIGHT_BOTTOM        3
#define QT_NODE_COUNT       4
#define QT_ELEM_MIN_SIZE    8

namespace Recast
{
    template<typename ValueType>
    class QuadTree
    {
    public:
        class Element
        {
        public:
            Element(ValueType* value)
            :mNode(nullptr)
            ,mValue(value)
            {
                
            }
            
            inline void* GetNode() const
            {
                return mNode;
            }
            
            inline void SetNode(void* node)
            {
                mNode = node;
            }
            
            inline ValueType* GetValue()
            {
                return mValue;
            }
            
            inline void SetValue(ValueType* value)
            {
                mValue = value;
            }
            
            inline const AABB* GetAABB() const
            {
                if (!mValue)
                    return nullptr;
                return mValue->GetAABB();
            }
            
            inline bool IsContain(float x, float y) const
            {
                if (!mValue)
                    return false;
                return mValue->IsContain(x, y);
            }
            
            inline bool Intersect(const Circle& circle) const
            {
                if (!mValue)
                    return false;
                return mValue->Intersect(circle);
            }
            
        private:
            void* mNode;
            ValueType* mValue;
        };
        
        class QuadNode
        {
        public:
            QuadNode(int deep, const AABB& aabb, QuadNode* parent)
            :mDeep(deep)
            ,mRouteElemCount(0)
            ,mAABB(aabb)
            ,mParent(parent)
            {
                memset(mChildren, 0, sizeof(mChildren));
            }
            
            ~QuadNode()
            {
                for (int i = 0; i < mElems.size(); ++i)
                {
                    delete mElems[i];
                }
                if (HasChild())
                {
                    for (int i = 0; i < QT_NODE_COUNT; ++i)
                    {
                        delete mChildren[i];
                    }
                }
            }
            
            inline int GetDeep() const
            {
                return mDeep;
            }

            inline bool HasChild() const
            {
                return !!mChildren[0];
            }
            
            inline float GetWidth() const
            {
                return mAABB.GetWidth();
            }
            
            inline float GetHeight() const
            {
                return mAABB.GetHeight();
            }
            
            inline float GetChildWidth() const
            {
                if (!mChildren[0])
                    return 0;
                return mChildren[0]->mAABB.GetWidth();
            }
            
            inline float GetChildHeight() const
            {
                if (!mChildren[0])
                    return 0;
                return mChildren[0]->mAABB.GetHeight();
            }

            inline int GetElemCount() const
            {
                return mElems.size();
            }

            inline int GetRouteElemCount() const
            {
                return mRouteElemCount;
            }

            void SetRouteElemCount(bool add)
            {
                QuadNode* node = this;
                while (node)
                {
                    if (add)
                        ++node->mRouteElemCount;
                    else
                        --node->mRouteElemCount;
                    node = node->mParent;
                }
            }

            inline const AABB& GetAABB() const
            {
                return mAABB;
            }

            inline QuadNode* GetParent() const
            {
                return mParent;
            }

            inline void InitChildren()
            {
                if (HasChild())
                    return;
                float x = mAABB.GetLeft();
                float y = mAABB.GetBottom();
                float width = mAABB.GetWidth();
                float height = mAABB.GetHeight();
                float halfWidth = width * 0.5f;
                float halfHeight = height * 0.5f;
                float midX = x + halfWidth;
                float midY = y + halfHeight;
                int deep = mDeep + 1;
                AABB aabb(0, 0, halfWidth, halfHeight);

                aabb.SetXY(midX, midY);
                mChildren[RIGHT_TOP] = new QuadNode(deep, aabb, this);

                aabb.SetXY(x, midY);
                mChildren[LEFT_TOP] = new QuadNode(deep, aabb, this);

                aabb.SetXY(x, y);
                mChildren[LEFT_BOTTOM] = new QuadNode(deep, aabb, this);

                aabb.SetXY(midX, y);
                mChildren[RIGHT_BOTTOM] = new QuadNode(deep, aabb, this);
            }

            inline QuadNode* GetChild(int pos) const
            {
                if (pos < 0 || pos >= QT_NODE_COUNT)
                    return nullptr;
                return mChildren[pos];
            }

            inline bool IsContain(const AABB& aabb) const
            {
                return mAABB.IsContain(aabb);
            }

            inline bool IsContain(float x, float y) const
            {
                return mAABB.IsContain(x, y);
            }
            
            inline bool Intersect(const AABB& aabb) const
            {
                return mAABB.Intersect(aabb);
            }

            Element* GetElem(int pos) const
            {
                if (pos < 0 || pos >= mElems.size())
                    return nullptr;
                return mElems[pos];
            }

            void AddElem(Element* elem)
            {
                QuadNode* node = (QuadNode*)elem->GetNode();
                if (node == this)
                    return;
                if (node)
                    node->RemoveElem(elem);
                elem->SetNode(this);
                mElems.push_back(elem);
                SetRouteElemCount(true);
            }

            void RemoveElem(Element* elem)
            {
                QuadNode* node = (QuadNode*)elem->GetNode();
                if (node != this)
                    return;
                for (int i = 0; i < mElems.size(); ++i)
                {
                    if (mElems[i] == elem)
                    {
                        SetRouteElemCount(false);
                        elem->SetNode(nullptr);
                        mElems.erase(mElems.begin() + i);
                        return;
                    }
                }
                elem->SetNode(nullptr);
            }
            
        private:
            int mDeep;
            int mRouteElemCount;
            AABB mAABB;
            QuadNode* mParent;
            QuadNode* mChildren[QT_NODE_COUNT];
            std::vector<Element*> mElems;
        };
        
    public:
        QuadTree(int maxDeep = 6)
        :mMaxDeep(maxDeep)
        ,mRoot(nullptr)
        {}
        
        QuadTree(float x, float y, float width, float height, int maxDeep = 6)
        :mMaxDeep(maxDeep)
        ,mRoot(nullptr)
        {
            Init(x, y, width, height);
        }
        
        ~QuadTree()
        {
            Clear();
        }
        
        inline bool IsInited()
        {
            return !!mRoot;
        }
        
        void Init(float x, float y, float width, float height)
        {
            Clear();
            AABB aabb(x, y, width, height);
            mRoot = new QuadNode(1, aabb, nullptr);
        }
        
        void Clear()
        {
            if (mRoot)
            {
                delete mRoot;
                mRoot = nullptr;
            }
        }
        
        bool Add(QuadNode* node, Element* elem, int atDeep = 0)
        {
            int curDeep = node->GetDeep();
            const AABB* aabb = elem->GetAABB();
            if ((!aabb || !node->IsContain(*aabb)) && curDeep > 1)
                return false;
            if (atDeep && atDeep == curDeep)
            {
                node->AddElem(elem);
                return true;
            }
            float width = node->GetWidth();
            float height = node->GetHeight();
            float childWidth = width * 0.5f;
            float childHeight = height * 0.5f;
            if (childWidth < aabb->GetWidth() ||
                childHeight < aabb->GetHeight())
            {
                node->AddElem(elem);
                return true;
            }
            if (node->HasChild())
            {
                for (int i = 0; i < QT_NODE_COUNT; ++i)
                {
                    auto* child = node->GetChild(i);
                    if (Add(child, elem, atDeep))
                        return true;
                }
                node->AddElem(elem);
                return true;
            }
            if (!atDeep)
            {
                if (curDeep >= mMaxDeep || node->GetElemCount() < QT_ELEM_MIN_SIZE)
                {
                    node->AddElem(elem);
                    return true;
                }
            }

            node->InitChildren();
            if (!atDeep)
            {
                for (int i = 0; i < QT_NODE_COUNT; ++i)
                {
                    QuadNode* child = node->GetChild(i);
                    int elemSize = node->GetElemCount();
                    for (int j = elemSize - 1; j >= 0; --j)
                    {
                        Element* e = node->GetElem(j);
                        Add(child, e);
                    }
                }
            }
            for (int i = 0; i < QT_NODE_COUNT; ++i)
            {
                QuadNode* child = node->GetChild(i);
                if (Add(child, elem, atDeep))
                    return true;
            }

            node->AddElem(elem);
            return true;
        }

        inline bool Add(Element* elem, int atDeep = 0)
        {
            if (!mRoot)
                return false;
            return Add(mRoot, elem, atDeep);
        }
        
        inline Element* Add(ValueType* value, int atDeep = 0)
        {
            if (!mRoot)
                return nullptr;
            Element* elem = new Element(value);
            Add(mRoot, elem, atDeep);
            return elem;
        }
        
        inline void Remove(Element* elem, bool del = true)
        {
            if (!elem)
                return;
            QuadNode* node = (QuadNode*)elem->GetNode();
            if (!node)
            {
				if (del)
					delete elem;
                return;
            }
            node->RemoveElem(elem);
			if (del)
				delete elem;
        }

        void Refresh(Element* elem)
        {
            QuadNode* node = (QuadNode*)elem->GetNode();
            if (!node || !node->IsContain(*elem->GetAABB()))
            {
                Add(elem);
                return;
            }
            Add(node, elem);
        }
        
        bool Intersect(float x, float y, std::vector<ValueType*>& output, bool getOne = false)
        {
            if (!mRoot)
                return false;
            return Intersect(mRoot, x, y, output, getOne);
        }
        
        bool Intersect(QuadNode* node, float x, float y, std::vector<ValueType*>& output, bool getOne = false)
        {
            if (!node->GetRouteElemCount())
                return false;
            if (!node->IsContain(x, y))
                return false;
            int elemCount = node->GetElemCount();
            bool hasValue = false;
            for (int i = 0; i < elemCount; ++i)
            {
                Element* elem = node->GetElem(i);
                ValueType* value = elem->GetValue();
                if (!value->IsContain(x, y))
                    continue;
                output.push_back(value);
                if (getOne)
                    return true;
                hasValue = true;
            }
            if (!node->HasChild())
                return hasValue;
            for (int i = 0; i < QT_NODE_COUNT; ++i)
            {
                QuadNode* child = node->GetChild(i);
                hasValue = Intersect(child, x, y, output, getOne) || hasValue;
                if (getOne && hasValue)
                    return true;
            }
            return hasValue;
        }
        
        bool Intersect(const Circle& circle, std::vector<ValueType*>& output, bool getOne = false)
        {
            if (!mRoot)
                return false;
            float radius = circle.GetRadius();
            float len = radius * 2;
            AABB aabb(circle.GetX() - radius,
                      circle.GetY() - radius,
                      len, len);
            return Intersect(mRoot, aabb, circle, output, getOne);
        }
        
        bool Intersect(QuadNode* node, const AABB& aabb, const Circle& circle, std::vector<ValueType*>& output, bool getOne = false)
        {
            if (!node->GetRouteElemCount())
                return false;
            if (!node->Intersect(aabb))
                return false;
            int elemCount = node->GetElemCount();
            bool hasValue = false;
            for (int i = 0; i < elemCount; ++i)
            {
                Element* elem = node->GetElem(i);
                ValueType* value = elem->GetValue();
                if (!value->Intersect(circle))
                    continue;
                output.push_back(value);
                if (getOne)
                    return true;
                hasValue = true;
            }
            if (!node->HasChild())
                return hasValue;
            for (int i = 0; i < QT_NODE_COUNT; ++i)
            {
                QuadNode* child = node->GetChild(i);
                hasValue = Intersect(child, aabb, circle, output, getOne) || hasValue;
                if (getOne && hasValue)
                    return true;
            }
            return hasValue;
        }
        
    private:
        int mMaxDeep;
        QuadNode* mRoot;
    };
}

#endif // QUAD_TREE_H
