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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <cmath>
#include <algorithm>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "ConvexVolumeTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "Filelist.h"
#include "QuadTree.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Quick and dirty convex hull.

enum ConvexCreation
{
    CONVEX_CREATION_REGION,
    CONVEX_CREATION_DOOR,
};

const int sIdMin = 1;
const int sIdMax = 100;
const int sBufferMaxSize = 1024;
const char* sVolumeTag = "Volume:";
const char* sAreaTag = "\tarea:";
const char* sHminTag = "\thmin:";
const char* sHmaxTag = "\thmax:";
const char* sNvertsTag = "\tnverts:";
const char* sVertTag = "\t\tvert:";
const char* sNlinkTag = "\tnlink:";
const char* sLinkTag = "\t\tlink:";
const char* sRegionTreeTag = "RegionTree:";
const char* sAABBTag = "\tAABB:";
const char* sTreeDeepTag = "\tDeep:";

// Returns true if 'c' is left of line 'a'-'b'.
inline bool left(const float* a, const float* b, const float* c)
{ 
	const float u1 = b[0] - a[0];
	const float v1 = b[2] - a[2];
	const float u2 = c[0] - a[0];
	const float v2 = c[2] - a[2];
	return u1 * v2 - v1 * u2 < 0;
}

// Returns true if 'a' is more lower-left than 'b'.
inline bool cmppt(const float* a, const float* b)
{
	if (a[0] < b[0]) return true;
	if (a[0] > b[0]) return false;
	if (a[2] < b[2]) return true;
	if (a[2] > b[2]) return false;
	return false;
}
// Calculates convex hull on xz-plane of points on 'pts',
// stores the indices of the resulting hull in 'out' and
// returns number of points on hull.
static int convexhull(const float* pts, int npts, int* out)
{
	// Find lower-leftmost point.
	int hull = 0;
	for (int i = 1; i < npts; ++i)
		if (cmppt(&pts[i*3], &pts[hull*3]))
			hull = i;
	// Gift wrap hull.
	int endpt = 0;
	int i = 0;
	do
	{
		out[i++] = hull;
		endpt = 0;
		for (int j = 1; j < npts; ++j)
			if (hull == endpt || left(&pts[hull*3], &pts[endpt*3], &pts[j*3]))
				endpt = j;
		hull = endpt;
	}
	while (endpt != out[0]);
	
	return i;
}

static int pointInPoly(int nvert, const float* verts, const float* p)
{
//    int i, j, c = 0;
//    for (i = 0, j = nvert-1; i < nvert; j = i++)
//    {
//        const float* vi = &verts[i*3];
//        const float* vj = &verts[j*3];
//        if (((vi[2] > p[2]) != (vj[2] > p[2])) &&
//            (p[0] < (vj[0]-vi[0]) * (p[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
//            c = !c;
//    }
//    return c;
    
    for (int i = 0, j = nvert-1; i < nvert; j = i++)
    {
        const float* a = verts + (j * 3);
        const float* b = verts + (i * 3);
        const float cx = p[0] - a[0];
        const float cz = p[2] - a[2];
        const float bx = b[0] - a[0];
        const float bz = b[2] - a[2];
        const float result = cz * bx - cx * bz;
        if (result <= 0)
            return 0;
    }
    return 1;
}

static void sortLinks(int* links, int linkSize)
{
    if (linkSize < 2)
        return;
    for (int i = 0; i < linkSize - 1; ++i)
    {
        for (int j = i + 1; j < linkSize; ++j)
        {
            int volumeIdI = getLinkVolumeId(links[i]);
            int volumeIdJ = getLinkVolumeId(links[j]);
            if (volumeIdI <= volumeIdJ)
                continue;
            int tmp = links[i];
            links[i] = links[j];
            links[j] = tmp;
        }
    }
}

ConvexVolumeTool::ConvexVolumeTool() :
	m_sample(0),
    m_creationType(CONVEX_CREATION_DOOR),
    m_id(1),
    m_linkId(1),
    m_doorId(0),
    m_autoIncrId(true),
	m_areaType(SAMPLE_POLYAREA_DOOR),
	m_polyOffset(0.0f),
	m_boxHeight(3.0f),
	m_boxDescent(1.0f),
    m_xSize(3.0f),
    m_ySize(3.0f),
    m_zSize(3.0f),
    m_rotation(0),
	m_npts(0),
	m_nhull(0)
{
}

void ConvexVolumeTool::init(Sample* sample)
{
	m_sample = sample;
}

void ConvexVolumeTool::reset()
{
	m_npts = 0;
	m_nhull = 0;
}

void ConvexVolumeTool::handleMenu()
{
    imguiLabel("Creation Type");
    if (imguiCheck("Region", m_creationType == CONVEX_CREATION_REGION))
    {
        m_creationType = CONVEX_CREATION_REGION;
        m_areaType = SAMPLE_POLYAREA_REGION;
        m_npts = 0;
        m_nhull = 0;
        m_error.clear();
    }
    if (imguiCheck("Door", m_creationType == CONVEX_CREATION_DOOR))
    {
        m_creationType = CONVEX_CREATION_DOOR;
        m_areaType = SAMPLE_POLYAREA_DOOR;
        m_npts = 0;
        m_nhull = 0;
        m_error.clear();
    }
    
    imguiSeparator();
    
    if (!m_error.empty())
        imguiLabel(m_error.c_str(), 255, 255, 0, 255);
    
    if (imguiCheck("Auto increase ID", m_autoIncrId))
        m_autoIncrId = !m_autoIncrId;
    
    const float idMin = (float)sIdMin;
    const float idMax = (float)sIdMax;
    imguiSlider("ID", &m_id, idMin, idMax, 1.0f);
    if (imguiButton("ID + 1"))
        m_id += 1.0f;
    if (imguiButton("ID - 1"))
        m_id -= 1.0f;
    m_id = m_id < idMin ? idMin : m_id > idMax ? idMax : m_id;
    
    if (m_creationType == CONVEX_CREATION_REGION)
    {
        imguiSlider("Shape Height", &m_boxHeight, 0.1f, 20.0f, 0.1f);
        imguiSlider("Shape Descent", &m_boxDescent, 0.1f, 20.0f, 0.1f);
//        imguiSlider("Poly Offset", &m_polyOffset, 0.0f, 10.0f, 0.1f);
        
        imguiSeparator();
        
        const ConvexVolume* fromVol = findRegion(m_id);
        const ConvexVolume* toVol = findRegion(m_linkId);
        if (!fromVol || !toVol || m_id == m_linkId)
        {
            imguiButton("Link area", false);
        }
        else
        {
            bool linked = false;
            for (int i = 0; i < fromVol->linkCount; ++i)
            {
                int volumeId = getLinkVolumeId(fromVol->links[i]);
                if (volumeId == m_linkId)
                {
                    linked = true;
                    break;
                }
            }
            if (linked)
            {
                if (imguiButton("Unlink area"))
                {
                    unlinkRegion(m_id, m_linkId, false);
                }
            }
            else
            {
                if (imguiButton("Link area"))
                {
                    linkRegion(m_id, m_linkId, m_doorId);
                }
            }
        }
        
        imguiSlider("Link ID", &m_linkId, idMin, idMax, 1.0f);
        if (imguiButton("Link ID + 1"))
            m_linkId += 1.0f;
        if (imguiButton("Link ID - 1"))
            m_linkId -= 1.0f;
        m_linkId = m_linkId < idMin ? idMin : m_linkId > idMax ? idMax : m_linkId;
        
        const int doorIdMin = idMin - 1;
        imguiSlider("Door ID", &m_doorId, doorIdMin, idMax, 1.0f);
        if (imguiButton("Door ID + 1"))
            m_doorId += 1.0f;
        if (imguiButton("Door ID - 1"))
            m_doorId -= 1.0f;
        m_doorId = m_doorId < doorIdMin ? doorIdMin : m_doorId > idMax ? idMax : m_doorId;
    }
	else if (m_creationType == CONVEX_CREATION_DOOR)
    {
        const float sizeMin = 1.0f;
        const float sizeMax = 100.0f;
        
        imguiSlider("X Size", &m_xSize, sizeMin, sizeMax, 0.1f);
        if (imguiButton("X + 0.1"))
            m_xSize += 0.1f;
        if (imguiButton("X - 0.1"))
            m_xSize -= 0.1f;
        m_xSize = m_xSize < sizeMin ? sizeMin : m_xSize > sizeMax ? sizeMax : m_xSize;
        
        imguiSlider("Y Size", &m_ySize, sizeMin, sizeMax, 0.1f);
        if (imguiButton("Y + 0.1"))
            m_ySize += 0.1f;
        if (imguiButton("Y - 0.1"))
            m_ySize -= 0.1f;
        m_ySize = m_ySize < sizeMin ? sizeMin : m_ySize > sizeMax ? sizeMax : m_ySize;
    
        imguiSlider("Z Size", &m_zSize, sizeMin, sizeMax, 0.1f);
        if (imguiButton("Z + 0.1"))
            m_zSize += 0.1f;
        if (imguiButton("Z - 0.1"))
            m_zSize -= 0.1f;
        m_zSize = m_zSize < sizeMin ? sizeMin : m_zSize > sizeMax ? sizeMax : m_zSize;
        
        const float rotationMin = 0.0f;
        const float rotationMax = 360.0f;
        imguiSlider("Rotation", &m_rotation, rotationMin, rotationMax, 0.1f);
        if (imguiButton("Rotation + 0.1"))
            m_rotation += 0.1f;
        if (imguiButton("Rotation - 0.1"))
            m_rotation -= 0.1f;
        m_rotation = m_rotation < rotationMin ? rotationMin : m_rotation > rotationMax ? rotationMax : m_rotation;
    }

	imguiSeparator();
    
    //    imguiLabel("Area Type");
    //    imguiIndent();
    //    if (imguiCheck("Ground", m_areaType == SAMPLE_POLYAREA_GROUND))
    //        m_areaType = SAMPLE_POLYAREA_GROUND;
    //    if (imguiCheck("Water", m_areaType == SAMPLE_POLYAREA_WATER))
    //        m_areaType = SAMPLE_POLYAREA_WATER;
    //    if (imguiCheck("Road", m_areaType == SAMPLE_POLYAREA_ROAD))
    //        m_areaType = SAMPLE_POLYAREA_ROAD;
    //    if (imguiCheck("Door", m_areaType == SAMPLE_POLYAREA_DOOR))
    //        m_areaType = SAMPLE_POLYAREA_DOOR;
    //    if (imguiCheck("Grass", m_areaType == SAMPLE_POLYAREA_GRASS))
    //        m_areaType = SAMPLE_POLYAREA_GRASS;
    //    if (imguiCheck("Jump", m_areaType == SAMPLE_POLYAREA_JUMP))
    //        m_areaType = SAMPLE_POLYAREA_JUMP;
    //    imguiUnindent();

	// imguiSeparator();

	if (imguiButton("Clear Shape"))
	{
		m_npts = 0;
		m_nhull = 0;
	}
    
    if (m_creationType == CONVEX_CREATION_REGION)
    {
        if (imguiButton("Save Regions"))
        {
            saveRegions();
        }
        
        if (imguiButton("Load Regions"))
        {
            loadRegions();
        }
    }
    else if (m_creationType == CONVEX_CREATION_DOOR)
    {
        if (imguiButton("Save Doors"))
        {
            saveDoors();
        }
        
        if (imguiButton("Load Doors"))
        {
            loadDoors();
        }
    }
}

void ConvexVolumeTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	
	if (shift)
	{
        if (m_npts && rcVdistSqr(p, &m_pts[(m_npts-1)*3]) < rcSqr(1.0f))
        {
            m_npts--;
            // Update hull.
            if (m_npts > 1)
                m_nhull = convexhull(m_pts, m_npts, m_hull);
            else
                m_nhull = 0;
            return;
        }
		// Delete
		int nearestIndex = -1;
		const ConvexVolume* vols = geom->getConvexVolumes();
        const ConvexVolume* vol = nullptr;
		for (int i = 0; i < geom->getConvexVolumeCount(); ++i)
		{
			if (pointInPoly(vols[i].nverts, vols[i].verts, p) &&
							p[1] >= vols[i].hmin && p[1] <= vols[i].hmax)
			{
				nearestIndex = i;
                vol = vols + i;
			}
		}
		// If end point close enough, delete it.
		if (nearestIndex != -1)
		{
            for (int i = 0; i < vol->linkCount; ++i)
            {
                int volumeId = getLinkVolumeId(vol->links[i]);
                unlinkRegion(vol->id, volumeId, true);
            }
			geom->deleteConvexVolume(nearestIndex);
		}
	}
	else
	{
		// Create
        int ret = ADD_CONVEX_SUCCESS;
        int id = (int)(m_id + 0.3f);
        if (m_creationType == CONVEX_CREATION_REGION)
        {
            // If clicked on that last pt, create the shape.
            if (m_npts && rcVdistSqr(p, &m_pts[(m_npts-1)*3]) < rcSqr(0.2f))
            {
                if (m_nhull > 2)
                {
                    // Create shape.
                    float verts[MAX_PTS*3];
                    for (int i = 0; i < m_nhull; ++i)
                        rcVcopy(&verts[i*3], &m_pts[m_hull[i]*3]);
                    
                    float minh = FLT_MAX, maxh = 0;
                    for (int i = 0; i < m_nhull; ++i)
                        minh = rcMin(minh, verts[i*3+1]);
                    minh -= m_boxDescent;
                    maxh = minh + m_boxHeight;
                    
                    if (m_polyOffset > 0.01f)
                    {
                        float offset[MAX_PTS*2*3];
                        int noffset = rcOffsetPoly(verts, m_nhull, m_polyOffset, offset, MAX_PTS*2);
                        if (noffset > 0)
                            ret = addConvexVolume(id, offset, noffset, minh, maxh, (unsigned char)m_areaType);
                    }
                    else
                    {
                        ret = addConvexVolume(id, verts, m_nhull, minh, maxh, (unsigned char)m_areaType);
                    }
                    if (ret == ADD_CONVEX_SUCCESS)
                    {
                        m_npts = 0;
                        m_nhull = 0;
                    }
                }
            }
            else
            {
                // Add new point
                if (m_npts < MAX_PTS)
                {
                    rcVcopy(&m_pts[m_npts*3], p);
                    m_npts++;
                    // Update hull.
                    if (m_npts > 1)
                        m_nhull = convexhull(m_pts, m_npts, m_hull);
                    else
                        m_nhull = 0;
                }
            }
        }
        else if (m_creationType == CONVEX_CREATION_DOOR)
        {
            /*
             2    1
             ------
             |    |
             |    |
             ------
             3    0
             */
            const float PI = 3.14159f;
            // x vector
            float aX = cos(m_rotation * PI / 180.0f);
            float aZ = sin(m_rotation * PI / 180.0f);
            float bX = cos((m_rotation + 90.0f) * PI / 180.0f);
            float bZ = sin((m_rotation + 90.0f) * PI / 180.0f);
            float* pt = nullptr;
            
            pt = &m_pts[0];
            rcVcopy(pt, p);
            pt[0] += (aX * m_xSize - bX * m_zSize) * 0.5f;
            pt[2] += (aZ * m_xSize - bZ * m_zSize) * 0.5f;
            
            pt = &m_pts[3];
            rcVcopy(pt, p);
            pt[0] += (aX * m_xSize + bX * m_zSize) * 0.5f;
            pt[2] += (aZ * m_xSize + bZ * m_zSize) * 0.5f;
            
            pt = &m_pts[6];
            rcVcopy(pt, p);
            pt[0] += (-aX * m_xSize + bX * m_zSize) * 0.5f;
            pt[2] += (-aZ * m_xSize + bZ * m_zSize) * 0.5f;
            
            pt = &m_pts[9];
            rcVcopy(pt, p);
            pt[0] += (-aX * m_xSize - bX * m_zSize) * 0.5f;
            pt[2] += (-aZ * m_xSize - bZ * m_zSize) * 0.5f;
            
            float minh = p[1] - m_ySize * 0.5f;
            float maxh = p[1] + m_ySize * 0.5f;
            
            ret = addConvexVolume(id, m_pts, 4, minh, maxh, (unsigned char)m_areaType);
        }
        if (ret == ADD_CONVEX_SUCCESS)
        {
            m_error = "";
        }
        else
        {
            char error[256];
            if (ret == ADD_CONVEX_EXIST_ID)
            {
                sprintf(error, "Error:Exist id %d", id);
            }
            m_error = error;
        }
	}
}

int ConvexVolumeTool::addConvexVolume(int id, const float* verts, const int nverts,
                    const float minh, const float maxh, unsigned char area)
{
    InputGeom* geom = m_sample->getInputGeom();
    int ret = geom->addConvexVolume(id, verts, nverts, minh, maxh, area);
    if (ret == ADD_CONVEX_SUCCESS)
    {
        if (m_autoIncrId)
            m_id = id + 1;
        return ADD_CONVEX_SUCCESS;
    }
    
    if (!m_autoIncrId)
        return ret;
    
    id = sIdMin;
    do
    {
        ret = geom->addConvexVolume(id, verts, nverts, minh, maxh, area);
        if (ret == ADD_CONVEX_SUCCESS)
        {
            if (m_autoIncrId)
                m_id = id + 1;
            break;
        }
        ++id;
        if (id > sIdMax)
            break;
    } while (ret != ADD_CONVEX_SUCCESS);
    return ret;
}

void ConvexVolumeTool::handleToggle()
{
}

void ConvexVolumeTool::handleStep()
{
}

void ConvexVolumeTool::handleUpdate(const float /*dt*/)
{
}

void ConvexVolumeTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();
	
	// Find height extent of the shape.
	float minh = FLT_MAX, maxh = 0;
	for (int i = 0; i < m_npts; ++i)
		minh = rcMin(minh, m_pts[i*3+1]);
	minh -= m_boxDescent;
	maxh = minh + m_boxHeight;

	dd.begin(DU_DRAW_POINTS, 4.0f);
	for (int i = 0; i < m_npts; ++i)
	{
		unsigned int col = duRGBA(255,255,255,255);
		if (i == m_npts-1)
			col = duRGBA(240,32,16,255);
		dd.vertex(m_pts[i*3+0],m_pts[i*3+1]+0.1f,m_pts[i*3+2], col);
	}
	dd.end();

	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0, j = m_nhull-1; i < m_nhull; j = i++)
	{
		const float* vi = &m_pts[m_hull[j]*3];
		const float* vj = &m_pts[m_hull[i]*3];
		dd.vertex(vj[0],minh,vj[2], duRGBA(255,255,255,64));
		dd.vertex(vi[0],minh,vi[2], duRGBA(255,255,255,64));
		dd.vertex(vj[0],maxh,vj[2], duRGBA(255,255,255,64));
		dd.vertex(vi[0],maxh,vi[2], duRGBA(255,255,255,64));
		dd.vertex(vj[0],minh,vj[2], duRGBA(255,255,255,64));
		dd.vertex(vj[0],maxh,vj[2], duRGBA(255,255,255,64));
	}
	dd.end();	
}

void ConvexVolumeTool::handleRenderOverlay(double* proj, double* model, int* view)
{
    renderVolumes(m_sample, proj, model, view);
    
	// Tool help
	const int h = view[3];
	if (!m_npts)
	{
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Create new shape.  SHIFT+LMB: Delete existing shape (click inside a shape).", imguiRGBA(255,255,255,192));	
	}
	else
	{
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "Click LMB to add new points. Click on the red point to finish the shape. SHIFT+LMB on the red point: delete point.", imguiRGBA(255,255,255,192));
		imguiDrawText(280, h-60, IMGUI_ALIGN_LEFT, "The shape will be convex hull of all added points.", imguiRGBA(255,255,255,192));	
	}	
}

void ConvexVolumeTool::saveVolumes(SamplePolyAreas area)
{
    InputGeom* geom = m_sample->getInputGeom();
    if (!geom)
        return;
    auto mesh = geom->getMesh();
    if (!mesh)
        return;
    const int maxSize = 1024;
    char buffer[maxSize];
    std::string volumeName = getFileName(mesh->getFileName());
    const char* fileExt = nullptr;
    switch (area)
    {
        case SAMPLE_POLYAREA_DOOR:
            fileExt = "door";
            break;
            
        case SAMPLE_POLYAREA_REGION:
            fileExt = "region";
            break;
        
        default: return;
    }
    snprintf(buffer, maxSize, "Output/%s.%s", volumeName.c_str(), fileExt);
    FILE* file = fopen(buffer, "w");
    const ConvexVolume* volumes = geom->getConvexVolumes();
    int volumeCount = geom->getConvexVolumeCount();
    std::vector<int> indices;
    for (int i = 0; i < volumeCount; ++i)
    {
        if (volumes[i].area != area)
            continue;
        indices.push_back(i);
    }
    if (indices.empty())
    {
        fclose(file);
        return;
    }
    if (indices.size() > 1)
    {
        std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            const ConvexVolume* va = volumes + a;
            const ConvexVolume* vb = volumes + b;
            return va->id < vb->id;
        });
    }
    std::vector<Recast::QuadTree<ConvexVolume>::Element*> elems;
    Recast::QuadTree<ConvexVolume> tree;
    if (area == SAMPLE_POLYAREA_REGION)
    {
        float minX = INFINITY;
        float maxX = -INFINITY;
        float minZ = INFINITY;
        float maxZ = -INFINITY;
        for (int i = 0; i < indices.size(); ++i)
        {
            ConvexVolume* v = (ConvexVolume*)volumes + indices[i];
            v->CalcAABB();
            for (int j = 0; j < v->nverts; ++j)
            {
                float x = v->verts[j * 3];
                minX = minX < x ? minX : x;
                maxX = maxX > x ? maxX : x;
                float z = v->verts[j * 3 + 2];
                minZ = minZ < z ? minZ : z;
                maxZ = maxZ > z ? maxZ : z;
            }
        }
        elems.resize(indices.size());
        float width = maxX - minX;
        float height = maxZ - minZ;
        tree.Init(minX - 1, minZ - 1, width + 1, height + 1);
        for (int i = 0; i < indices.size(); ++i)
        {
            ConvexVolume* v = (ConvexVolume*)volumes + indices[i];
            elems[i] = tree.Add(v);
        }
        int size = snprintf(buffer, maxSize, "%sx=%0.6f,y=%0.6f,width=%0.6f,height=%0.6f\n", sRegionTreeTag, minX, minZ, width, height);
        fwrite(buffer, 1, size, file);
    }
    for (int i = 0; i < indices.size(); ++i)
    {
        int size = 0;
        const int index = indices[i];
        const ConvexVolume* volume = &volumes[index];
        
        size = snprintf(buffer, maxSize, "%s%d\n", sVolumeTag, volume->id);
        fwrite(buffer, 1, size, file);
        
        size = snprintf(buffer, maxSize, "%s%d\n", sAreaTag, volume->area);
        fwrite(buffer, 1, size, file);
        
        if (area == SAMPLE_POLYAREA_REGION)
        {
            const Recast::AABB* aabb = volume->GetAABB();
            size = snprintf(buffer, maxSize, "%sx=%0.6f,y=%0.6f,width=%0.6f,height=%0.6f\n", sAABBTag, aabb->GetLeft(), aabb->GetBottom(), aabb->GetWidth(), aabb->GetHeight());
            fwrite(buffer, 1, size, file);
            
            auto* elem = elems[i];
            auto* node = (Recast::QuadTree<ConvexVolume>::QuadNode*)elem->GetNode();
            size = snprintf(buffer, maxSize, "%s%d\n", sTreeDeepTag, node->GetDeep());
            fwrite(buffer, 1, size, file);
        }
        
        size = snprintf(buffer, maxSize, "%s%0.6f\n", sHminTag, volume->hmin);
        fwrite(buffer, 1, size, file);
        
        size = snprintf(buffer, maxSize, "%s%0.6f\n", sHmaxTag, volume->hmax);
        fwrite(buffer, 1, size, file);
        
        size = snprintf(buffer, maxSize, "%s%d\n", sNvertsTag, volume->nverts);
        fwrite(buffer, 1, size, file);
        
        for (int j = 0; j < volume->nverts; ++j)
        {
            const float* verts = &volume->verts[j * 3];
            size = snprintf(buffer, maxSize, "%sx=%0.6f,y=%0.6f,z=%0.6f\n", sVertTag, verts[0], verts[1], verts[2]);
            fwrite(buffer, 1, size, file);
        }
        
        if (volume->linkCount > 0)
        {
            size = snprintf(buffer, maxSize, "%s%d\n", sNlinkTag, volume->linkCount);
            fwrite(buffer, 1, size, file);
            
            sortLinks((int*)volume->links, volume->linkCount);
            for (int j = 0; j < volume->linkCount; ++j)
            {
                size = snprintf(buffer, maxSize, "%s%d\n", sLinkTag, volume->links[j]);
                fwrite(buffer, 1, size, file);
            }
        }
    }
    fclose(file);
}

void ConvexVolumeTool::loadVolumes(SamplePolyAreas area)
{
    InputGeom* geom = m_sample->getInputGeom();
    if (!geom)
        return;
    auto mesh = geom->getMesh();
    if (!mesh)
        return;
    const int maxSize = sBufferMaxSize;
    char buffer1[maxSize];
    char buffer2[maxSize];
    const char* fileExt = nullptr;
    switch (area)
    {
        case SAMPLE_POLYAREA_DOOR:
            fileExt = "door";
            break;
            
        case SAMPLE_POLYAREA_REGION:
            fileExt = "region";
            break;
            
        default: return;
    }
    std::string volumeName = getFileName(mesh->getFileName());
    snprintf(buffer1, maxSize, "Output/%s.%s", volumeName.c_str(), fileExt);
    FILE* file = fopen(buffer1, "r");
    if (!file)
        return;

	auto readFunc = [&](char* buffer, int size) {
		return fread(buffer, 1, size, file);
	};

    geom->deleteConvexVolumes(area);
    char* buffer = buffer1;
    int bufferPos = 0;
    int readCount = 0;
    bool readEnd = false;
    ConvexVolume volume;
    volume.id = 0;
    int vertIndex = 0;
    int linkIndex = 0;
    const int volumeTagSize = strlen(sVolumeTag);
    const int areaTagSize = strlen(sAreaTag);
    const int hminTagSize = strlen(sHminTag);
    const int hmaxTagSize = strlen(sHmaxTag);
    const int nvertsTagSize = strlen(sNvertsTag);
    const int vertTagSize = strlen(sVertTag);
    const int nlinkTagSize = strlen(sNlinkTag);
    const int linkTagSize = strlen(sLinkTag);
    const int scanFormatSize = 256;
    char scanFormat[scanFormatSize];
    do
    {
        char* str = buffer == buffer1 ? buffer2 : buffer1;
        bool success = readLine(readFunc, buffer, maxSize, bufferPos, readCount, str, readEnd);
        if (!success && readEnd)
            break;
        if (strncmp(str, sVolumeTag, volumeTagSize) == 0)
        {
            if (volume.id)
            {
                geom->addConvexVolume(volume.id, volume.verts, volume.nverts, volume.hmin, volume.hmax, (unsigned char)volume.area, volume.linkCount, volume.links);
            }
            snprintf(scanFormat, scanFormatSize, "%s%%d", sVolumeTag);
            sscanf(str, scanFormat, &volume.id);
            volume.linkCount = 0;
            vertIndex = 0;
            linkIndex = 0;
            continue;
        }
        if (strncmp(str, sAreaTag, areaTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%d", sAreaTag);
            sscanf(str, scanFormat, &volume.area);
            continue;
        }
        if (strncmp(str, sHminTag, hminTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%f", sHminTag);
            sscanf(str, scanFormat, &volume.hmin);
            continue;
        }
        if (strncmp(str, sHmaxTag, hmaxTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%f", sHmaxTag);
            sscanf(str, scanFormat, &volume.hmax);
            continue;
        }
        if (strncmp(str, sNvertsTag, nvertsTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%d", sNvertsTag);
            sscanf(str, scanFormat, &volume.nverts);
            continue;
        }
        if (strncmp(str, sVertTag, vertTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%sx=%%f,y=%%f,z=%%f", sVertTag);
            float* vert = &volume.verts[vertIndex++ * 3];
            sscanf(str, scanFormat, &vert[0], &vert[1], &vert[2]);
            continue;
        }
        if (strncmp(str, sNlinkTag, nlinkTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%d", sNlinkTag);
            sscanf(str, scanFormat, &volume.linkCount);
            continue;
        }
        if (strncmp(str, sLinkTag, linkTagSize) == 0)
        {
            snprintf(scanFormat, scanFormatSize, "%s%%d", sLinkTag);
            sscanf(str, scanFormat, volume.links + linkIndex);
            ++linkIndex;
            continue;
        }
    } while (true);
    if (volume.id)
    {
        geom->addConvexVolume(volume.id, volume.verts, volume.nverts, volume.hmin, volume.hmax, (unsigned char)volume.area, volume.linkCount, volume.links);
    }
    fclose(file);
}

const ConvexVolume* ConvexVolumeTool::findRegion(int id)
{
    InputGeom* geom = m_sample->getInputGeom();
    if (!geom)
        return nullptr;
    ConvexVolume* volumes = (ConvexVolume*)geom->getConvexVolumes();
    int volumeCount = geom->getConvexVolumeCount();
    for (int i = 0; i < volumeCount; ++i)
    {
        ConvexVolume* volume = volumes + i;
        if (volume->area != SAMPLE_POLYAREA_REGION)
            continue;
        if (volume->id == id)
        {
            return volume;
        }
    }
    return nullptr;
}

void ConvexVolumeTool::linkRegion(int from, int to, int doorId)
{
    if (from == to)
        return;
    InputGeom* geom = m_sample->getInputGeom();
    if (!geom)
        return;
    ConvexVolume* fromVolume = nullptr;
    ConvexVolume* toVolume = nullptr;
    ConvexVolume* volumes = (ConvexVolume*)geom->getConvexVolumes();
    int volumeCount = geom->getConvexVolumeCount();
    for (int i = 0; i < volumeCount; ++i)
    {
        ConvexVolume* volume = volumes + i;
        if (volume->area != SAMPLE_POLYAREA_REGION)
            continue;
        if (volume->id == from)
        {
            fromVolume = volume;
            if (toVolume)
                break;
            continue;
        }
        if (volume->id == to)
        {
            toVolume = volume;
            if (fromVolume)
                break;
        }
    }
    if (!fromVolume)
    {
        m_error = "Cannot find from volume";
        return;
    }
    if (!toVolume)
    {
        m_error = "Cannot find to volume";
        return;
    }
    m_error = "";
    for (int i = 0; i < fromVolume->linkCount; ++i)
    {
        int volumeId = getLinkVolumeId(fromVolume->links[i]);
        if (volumeId == to)
        {
            m_error = "from volume linked to volume";
            return;
        }
    }
    for (int i = 0; i < toVolume->linkCount; ++i)
    {
        int volumeId = getLinkVolumeId(toVolume->links[i]);
        if (volumeId == from)
        {
            m_error = "to volume linked from volume";
            return;
        }
    }
    fromVolume->links[fromVolume->linkCount++] = buildLinkId(to, doorId);
    sortLinks(fromVolume->links, fromVolume->linkCount);

    toVolume->links[toVolume->linkCount++] = buildLinkId(from, doorId);
    sortLinks(toVolume->links, toVolume->linkCount);
}

void ConvexVolumeTool::unlinkRegion(int from, int to, bool ignoreFrom)
{
    if (from == to)
        return;
    InputGeom* geom = m_sample->getInputGeom();
    if (!geom)
        return;
    ConvexVolume* fromVolume = nullptr;
    ConvexVolume* toVolume = nullptr;
    ConvexVolume* volumes = (ConvexVolume*)geom->getConvexVolumes();
    int volumeCount = geom->getConvexVolumeCount();
    for (int i = 0; i < volumeCount; ++i)
    {
        ConvexVolume* volume = volumes + i;
        if (volume->area != SAMPLE_POLYAREA_REGION)
            continue;
        if (volume->id == from)
        {
            fromVolume = volume;
            if (toVolume)
                break;
            continue;
        }
        if (volume->id == to)
        {
            toVolume = volume;
            if (fromVolume)
                break;
        }
    }
    if (fromVolume && !ignoreFrom)
    {
        for (int i = 0; i < fromVolume->linkCount; ++i)
        {
            int volumeId = getLinkVolumeId(fromVolume->links[i]);
            if (volumeId == to)
            {
                fromVolume->links[i] = fromVolume->links[--fromVolume->linkCount];
                break;
            }
        }
        sortLinks(fromVolume->links, fromVolume->linkCount);
    }
    
    if (toVolume)
    {
        for (int i = 0; i < toVolume->linkCount; ++i)
        {
            int volumeId = getLinkVolumeId(toVolume->links[i]);
            if (volumeId == from)
            {
                toVolume->links[i] = toVolume->links[--toVolume->linkCount];
                break;
            }
        }
        sortLinks(toVolume->links, toVolume->linkCount);
    }
}
