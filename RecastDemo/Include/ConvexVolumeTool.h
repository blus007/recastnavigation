//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef CONVEXVOLUMETOOL_H
#define CONVEXVOLUMETOOL_H

#include <string>
#include "Sample.h"

// Tool to create convex volumess for InputGeom

class ConvexVolumeTool : public SampleTool
{
	Sample* m_sample;
    int m_creationType;
    float m_id;
    float m_linkId;
    float m_doorId;
    bool m_autoIncrId;
    std::string m_error;
    
	int m_areaType;
	float m_polyOffset;
	float m_boxHeight;
	float m_boxDescent;
    
    float m_xSize;
    float m_ySize;
    float m_zSize;
    float m_rotation;
	
	static const int MAX_PTS = 12;
	float m_pts[MAX_PTS*3];
	int m_npts;
	int m_hull[MAX_PTS];
	int m_nhull;
	
public:
	ConvexVolumeTool();
	
	virtual int type() { return TOOL_CONVEX_VOLUME; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
    
private:
    int addConvexVolume(const int id, const float* verts, const int nverts,
                        const float minh, const float maxh, unsigned char area);
    
    void saveVolumes(SamplePolyAreas area);
    void loadVolumes(SamplePolyAreas area);
    
    inline void saveDoors()
    {
        saveVolumes(SAMPLE_POLYAREA_DOOR);
    }
    inline void loadDoors()
    {
        loadVolumes(SAMPLE_POLYAREA_DOOR);
    }
    
    inline void saveRegions()
    {
        saveVolumes(SAMPLE_POLYAREA_REGION);
    }
    inline void loadRegions()
    {
        loadVolumes(SAMPLE_POLYAREA_REGION);
    }
    
    const struct ConvexVolume* findRegion(int id);
    void linkRegion(int from, int to, int doorId);
    void unlinkRegion(int from, int to, bool ignoreFrom);
};

#endif // CONVEXVOLUMETOOL_H
