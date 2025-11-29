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

#include <math.h>
#include <stdio.h>
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#    include <OpenGL/glu.h>
#else
#    include <GL/glu.h>
#endif

#include "Filelist.h"
#include "nlohmann/json.hpp"
#include "QuadTree.h"
#include <fstream>

#ifdef WIN32
#	define snprintf _snprintf
#endif

extern bool g_showBlock;
extern bool g_showBlockName;

SampleTool::~SampleTool()
{
	// Defined out of line to fix the weak v-tables warning
}

void SampleTool::renderVolumes(Sample* sample, double* proj, double* model, int* view)
{
    GLdouble x, y, z;
    
    InputGeom* geom = sample->getInputGeom();
    if (geom)
    {
        int volumeCount = geom->getConvexVolumeCount();
        if (volumeCount > 0)
        {
            const int BUFF_SIZE = 512;
            char buff[BUFF_SIZE];
			const std::list<ConvexVolume*>& vols = geom->getConvexVolumes();
			for (auto it = vols.begin(); it != vols.end(); ++it)
            {
                const ConvexVolume* vol = *it;
				if (vol->area == SAMPLE_POLYAREA_BLOCK && (!g_showBlock || !g_showBlockName))
					continue;
                const float* verts = vol->verts;
                const int nverts = vol->nverts;
                float centerPos[3] = {0,0,0};
                for (int i = 0; i < nverts; ++i)
                    dtVadd(centerPos,centerPos,&verts[i*3]);
                dtVscale(centerPos,centerPos,1.0f/nverts);
                if (gluProject((GLdouble)centerPos[0], (GLdouble)vol->hmax, (GLdouble)centerPos[2], model, proj, view, &x, &y, &z))
                {
                    const char* areaName = nullptr;
                    switch (vol->area)
                    {
                        case SAMPLE_POLYAREA_DOOR:
                            areaName = "door";
                            break;
                            
                        case SAMPLE_POLYAREA_REGION:
                            areaName = "region";
                            break;

						case SAMPLE_POLYAREA_BLOCK:
							areaName = "block";
							break;
                            
                        default:
                            areaName = "unknow";
                            break;
                    }
                    snprintf(buff, BUFF_SIZE, "%s:%d", areaName, vol->id);
                    if (vol->linkCount > 0)
                    {
                        imguiDrawText((int)x, (int)(y+8), IMGUI_ALIGN_CENTER, buff, imguiRGBA(0,0,0,220));
                        int size = snprintf(buff, BUFF_SIZE, "%s", "link:");
                        for (int j = 0; j < vol->linkCount; ++j)
                        {
                            if (j)
                                size += snprintf(buff + size, BUFF_SIZE - size, "%s", ",");
                            int volumeId = getLinkVolumeId(vol->links[j]);
                            int doorId = getLinkDoorId(vol->links[j]);
                            if (doorId > 0)
                                size += snprintf(buff + size, BUFF_SIZE - size, "%d-%d", volumeId, doorId);
                            else
                                size += snprintf(buff + size, BUFF_SIZE - size, "%d", volumeId);
                        }
                        imguiDrawText((int)x, (int)(y-8), IMGUI_ALIGN_CENTER, buff, imguiRGBA(0,0,0,220));
                    }
                    else
                    {
                        imguiDrawText((int)x, (int)(y), IMGUI_ALIGN_CENTER, buff, imguiRGBA(0,0,0,220));
                    }
                }
            }
        }
    }
}

SampleToolState::~SampleToolState()
{
	// Defined out of line to fix the weak v-tables warning
}

unsigned int SampleDebugDraw::areaToCol(unsigned int area)
{
	switch(area)
	{
	// Ground (0) : light blue
	case SAMPLE_POLYAREA_GROUND: return duRGBA(0, 192, 255, 255);
	// Water : blue
	case SAMPLE_POLYAREA_WATER: return duRGBA(0, 0, 255, 255);
	// Road : brown
	case SAMPLE_POLYAREA_ROAD: return duRGBA(50, 20, 12, 255);
	// Door : cyan
	case SAMPLE_POLYAREA_DOOR: return duRGBA(0, 255, 255, 255);
	// Grass : green
	case SAMPLE_POLYAREA_GRASS: return duRGBA(0, 255, 0, 255);
	// Jump : yellow
	case SAMPLE_POLYAREA_JUMP: return duRGBA(255, 255, 0, 255);
    // Region : light green 99CC66
    case SAMPLE_POLYAREA_REGION: return duRGBA(153, 204, 102, 255);
	// Block : gray
	case SAMPLE_POLYAREA_BLOCK: return duRGBA(128, 128, 128, 255);
	// Unexpected : red
	default: return duRGBA(255, 0, 0, 255);
	}
}

Sample::Sample() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);
	delete m_tool;
	for (int i = 0; i < MAX_TOOLS; i++)
		delete m_toolStates[i];
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;
	if (tool)
		m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_geom)
		return;
	
	// Draw mesh
	duDebugDrawTriMesh(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
					   m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.0f);
	// Draw bounds
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings)
	{
		m_cellSize = buildSettings->cellSize;
		m_cellHeight = buildSettings->cellHeight;
		m_agentHeight = buildSettings->agentHeight;
		m_agentRadius = buildSettings->agentRadius;
		m_agentMaxClimb = buildSettings->agentMaxClimb;
		m_agentMaxSlope = buildSettings->agentMaxSlope;
		m_regionMinSize = buildSettings->regionMinSize;
		m_regionMergeSize = buildSettings->regionMergeSize;
		m_edgeMaxLen = buildSettings->edgeMaxLen;
		m_edgeMaxError = buildSettings->edgeMaxError;
		m_vertsPerPoly = buildSettings->vertsPerPoly;
		m_detailSampleDist = buildSettings->detailSampleDist;
		m_detailSampleMaxError = buildSettings->detailSampleMaxError;
		m_partitionType = buildSettings->partitionType;
	}
}

void Sample::collectSettings(BuildSettings& settings)
{
	settings.cellSize = m_cellSize;
	settings.cellHeight = m_cellHeight;
	settings.agentHeight = m_agentHeight;
	settings.agentRadius = m_agentRadius;
	settings.agentMaxClimb = m_agentMaxClimb;
	settings.agentMaxSlope = m_agentMaxSlope;
	settings.regionMinSize = m_regionMinSize;
	settings.regionMergeSize = m_regionMergeSize;
	settings.edgeMaxLen = m_edgeMaxLen;
	settings.edgeMaxError = m_edgeMaxError;
	settings.vertsPerPoly = m_vertsPerPoly;
	settings.detailSampleDist = m_detailSampleDist;
	settings.detailSampleMaxError = m_detailSampleMaxError;
	settings.partitionType = m_partitionType;
}

void Sample::loadSettings(const struct BuildSettings& settings)
{
	m_cellSize = settings.cellSize;
	m_cellHeight = settings.cellHeight;
	m_agentHeight = settings.agentHeight;
	m_agentRadius = settings.agentRadius;
	m_agentMaxClimb = settings.agentMaxClimb;
	m_agentMaxSlope = settings.agentMaxSlope;
	m_regionMinSize = settings.regionMinSize;
	m_regionMergeSize = settings.regionMergeSize;
	m_edgeMaxLen = settings.edgeMaxLen;
	m_edgeMaxError = settings.edgeMaxError;
	m_vertsPerPoly = settings.vertsPerPoly;
	m_detailSampleDist = settings.detailSampleDist;
	m_detailSampleMaxError = settings.detailSampleMaxError;
	m_partitionType = settings.partitionType;
	m_filterLowHangingObstacles = settings.filterLowHangingObstacles;
	m_filterLedgeSpans = settings.filterLedgeSpans;
	m_filterWalkableLowHeightSpans = settings.filterWalkableLowHeightSpans;
}

void Sample::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 8;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &m_cellSize, 0.1f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &m_cellHeight, 0.1f, 1.0f, 0.01f);
	
	if (m_geom)
	{
		const float* bmin = m_geom->getNavMeshBoundsMin();
		const float* bmax = m_geom->getNavMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}
	
	imguiSeparator();
	imguiLabel("Agent");
	imguiSlider("Height", &m_agentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius", &m_agentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb", &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope", &m_agentMaxSlope, 0.0f, 90.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Region");
	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.0f);
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Partitioning");
	if (imguiCheck("Watershed", m_partitionType == SAMPLE_PARTITION_WATERSHED))
		m_partitionType = SAMPLE_PARTITION_WATERSHED;
	if (imguiCheck("Monotone", m_partitionType == SAMPLE_PARTITION_MONOTONE))
		m_partitionType = SAMPLE_PARTITION_MONOTONE;
	if (imguiCheck("Layers", m_partitionType == SAMPLE_PARTITION_LAYERS))
		m_partitionType = SAMPLE_PARTITION_LAYERS;
	
	imguiSeparator();
	imguiLabel("Filtering");
	if (imguiCheck("Low Hanging Obstacles", m_filterLowHangingObstacles))
		m_filterLowHangingObstacles = !m_filterLowHangingObstacles;
	if (imguiCheck("Ledge Spans", m_filterLedgeSpans))
		m_filterLedgeSpans= !m_filterLedgeSpans;
	if (imguiCheck("Walkable Low Height Spans", m_filterWalkableLowHeightSpans))
		m_filterWalkableLowHeightSpans = !m_filterWalkableLowHeightSpans;

	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.0f);
	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.0f);		

	imguiSeparator();
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 16.0f, 1.0f);
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

dtNavMesh* Sample::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

void Sample::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

extern void sortLinks(int* links, int linkSize);

void Sample::saveDoor()
{
	InputGeom* geom = getInputGeom();
	if (!geom)
		return;
	auto mesh = geom->getMesh();
	if (!mesh)
		return;
	const int maxSize = 1024;
	char buffer[maxSize];
	std::string volumeName = getFileName(mesh->getFileName());
	snprintf(buffer, maxSize, "Output/%s.door", volumeName.c_str());
	FILE* file = fopen(buffer, "w");
	const std::list<ConvexVolume*>& volumes = geom->getConvexVolumes();
	int volumeCount = geom->getConvexVolumeCount();
	const int area = SAMPLE_POLYAREA_DOOR;
	std::vector<const ConvexVolume*> doors;
	for (auto it = volumes.begin(); it != volumes.end(); ++it)
	{
		const ConvexVolume* vol = *it;
		if (vol->area != area)
			continue;
		doors.push_back(vol);
	}
	if (doors.empty())
	{
		fclose(file);
		return;
	}
	if (doors.size() > 1)
	{
		std::sort(doors.begin(), doors.end(), [&](const ConvexVolume* a, const ConvexVolume* b) {
			return a->id < b->id;
		});
	}
	nlohmann::json data;
	data["volumes"] = nlohmann::json::array();
	for (int i = 0; i < doors.size(); ++i)
	{
		int size = 0;
		const ConvexVolume* volume = doors[i];

		nlohmann::json item = nlohmann::json::object();
		item["id"] = volume->id;
		item["hmin"] = volume->hmin;
		item["hmax"] = volume->hmax;
		item["verts"] = nlohmann::json::array();
		for (int j = 0; j < volume->nverts; ++j)
		{
			const float* verts = &volume->verts[j * 3];
			item["verts"].push_back({ verts[0], verts[1], verts[2] });
		}
		if (volume->linkCount == 2)
		{
			sortLinks((int*)volume->links, volume->linkCount);
			item["link"] = { volume->links[0], volume->links[1] };
		}
		else
		{
			item["link"] = { 200, 201 };
		}
		data["volumes"].push_back(item);
	}
	std::string str = data.dump(2, ' ');
	fwrite(str.c_str(), str.size(), 1, file);
	fclose(file);
}

void Sample::loadDoor()
{
	InputGeom* geom = getInputGeom();
	if (!geom)
		return;
	auto mesh = geom->getMesh();
	if (!mesh)
		return;
	const int maxSize = 1024;
	char buffer[maxSize];
	const int area = SAMPLE_POLYAREA_DOOR;
	std::string volumeName = getFileName(mesh->getFileName());
	snprintf(buffer, maxSize, "Output/%s.door", volumeName.c_str());
	std::ifstream read(buffer);
	if (!read.is_open())
		return;
	nlohmann::json data = nlohmann::json::parse(read);
	geom->deleteConvexVolumes(area);
	float verts[MAX_CONVEXVOL_PTS * 3];
	int links[2];
	const int volumeCount = data["volumes"].size();
	for (int i = 0; i < volumeCount; ++i)
	{
		auto& volume = data["volumes"][i];
		const int vertCount = volume["verts"].size();
		for (int j = 0; j < vertCount; ++j)
		{
			auto& jvert = volume["verts"][j];
			verts[j * 3] = jvert[0];
			verts[j * 3 + 1] = jvert[1];
			verts[j * 3 + 2] = jvert[2];
		}
		links[0] = volume["link"][0];
		links[1] = volume["link"][1];
		geom->addConvexVolume(volume["id"], verts, vertCount, volume["hmin"], volume["hmax"], (unsigned char)area, 2, links);
	}
}

void Sample::saveRegion()
{
	InputGeom* geom = getInputGeom();
	if (!geom)
		return;
	auto mesh = geom->getMesh();
	if (!mesh)
		return;
	const int maxSize = 1024;
	char buffer[maxSize];
	std::string volumeName = getFileName(mesh->getFileName());
	snprintf(buffer, maxSize, "Output/%s.region", volumeName.c_str());
	FILE* file = fopen(buffer, "w");
	const std::list<ConvexVolume*>& volumes = geom->getConvexVolumes();
	int volumeCount = geom->getConvexVolumeCount();
	const int area = SAMPLE_POLYAREA_REGION;
	std::vector<ConvexVolume*> regions;
	for (auto it = volumes.begin(); it != volumes.end(); ++it)
	{
		ConvexVolume* vol = *it;
		if (vol->area != area)
			continue;
		regions.push_back(vol);
	}
	if (regions.empty())
	{
		fclose(file);
		return;
	}
	if (regions.size() > 1)
	{
		std::sort(regions.begin(), regions.end(), [&](const ConvexVolume* a, const ConvexVolume* b) {
			return a->id < b->id;
		});
	}
	nlohmann::json data;
	data["info"] = nlohmann::json::object();
	data["volumes"] = nlohmann::json::array();
	std::vector<Recast::QuadTree<ConvexVolume>::Element*> elems;
	Recast::QuadTree<ConvexVolume> tree;
	{
		float minX = HUGE_VALF;
		float maxX = -HUGE_VALF;
		float minZ = HUGE_VALF;
		float maxZ = -HUGE_VALF;
		for (int i = 0; i < regions.size(); ++i)
		{
			ConvexVolume* v = regions[i];
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
		elems.resize(regions.size());
		float width = maxX - minX;
		float height = maxZ - minZ;
		tree.Init(minX - 1, minZ - 1, width + 1, height + 1);
		for (int i = 0; i < regions.size(); ++i)
		{
			ConvexVolume* v = regions[i];
			elems[i] = tree.Add(v);
		}
		data["info"]["x"] = minX;
		data["info"]["z"] = minZ;
		data["info"]["width"] = width;
		data["info"]["height"] = height;
	}
	for (int i = 0; i < regions.size(); ++i)
	{
		int size = 0;
		const ConvexVolume* volume = regions[i];

		nlohmann::json item = nlohmann::json::object();
		item["id"] = volume->id;
		item["province"] = 1;
		item["aabb"] = nlohmann::json::object();
		auto* node = (Recast::QuadTree<ConvexVolume>::QuadNode*)elems[i]->GetNode();
		item["deep"] = node->GetDeep();
		item["hmin"] = volume->hmin;
		item["hmax"] = volume->hmax;
		item["verts"] = nlohmann::json::array();
		float minX = HUGE_VALF;
		float maxX = -HUGE_VALF;
		float minZ = HUGE_VALF;
		float maxZ = -HUGE_VALF;
		for (int j = 0; j < volume->nverts; ++j)
		{
			const float* verts = &volume->verts[j * 3];
			float x = verts[j * 3];
			minX = minX < x ? minX : x;
			maxX = maxX > x ? maxX : x;
			float z = verts[j * 3 + 2];
			minZ = minZ < z ? minZ : z;
			maxZ = maxZ > z ? maxZ : z;
			item["verts"].push_back({ verts[0], verts[1], verts[2] });
		}
		float width = maxX - minX;
		float height = maxZ - minZ;
		item["aabb"]["x"] = minX;
		item["aabb"]["z"] = minZ;
		item["aabb"]["width"] = width;
		item["aabb"]["height"] = height;
		data["volumes"].push_back(item);
	}
	std::string str = data.dump(2, ' ');
	fwrite(str.c_str(), str.size(), 1, file);
	fclose(file);
}

void Sample::loadRegion()
{
	InputGeom* geom = getInputGeom();
	if (!geom)
		return;
	auto mesh = geom->getMesh();
	if (!mesh)
		return;
	const int maxSize = 1024;
	char buffer[maxSize];
	const int area = SAMPLE_POLYAREA_REGION;
	std::string volumeName = getFileName(mesh->getFileName());
	snprintf(buffer, maxSize, "Output/%s.region", volumeName.c_str());
	std::ifstream read(buffer);
	if (!read.is_open())
		return;
	nlohmann::json data = nlohmann::json::parse(read);
	geom->deleteConvexVolumes(area);
	float verts[MAX_CONVEXVOL_PTS * 3];
	const int volumeCount = data["volumes"].size();
	for (int i = 0; i < volumeCount; ++i)
	{
		auto& volume = data["volumes"][i];
		const int vertCount = volume["verts"].size();
		for (int j = 0; j < vertCount; ++j)
		{
			auto& jvert = volume["verts"][j];
			verts[j * 3] = jvert[0];
			verts[j * 3 + 1] = jvert[1];
			verts[j * 3 + 2] = jvert[2];
		}
		geom->addConvexVolume(volume["id"], verts, vertCount, volume["hmin"], volume["hmax"], (unsigned char)area, 0, nullptr);
	}
}

void Sample::loadBlock()
{
	InputGeom* geom = getInputGeom();
	if (!geom)
		return;
	auto mesh = geom->getMesh();
	if (!mesh)
		return;
	const int maxSize = 1024;
	char buffer[maxSize];
	const int area = SAMPLE_POLYAREA_BLOCK;
	std::string volumeName = getFileName(mesh->getFileName());
	snprintf(buffer, maxSize, "Output/%s.block", volumeName.c_str());
	std::ifstream read(buffer);
	if (!read.is_open())
		return;
	nlohmann::json data = nlohmann::json::parse(read);
	geom->deleteConvexVolumes(area);
	float verts[MAX_CONVEXVOL_PTS * 3];
	auto& info = data["info"];
	const int volumeCount = data["volumes"].size();
	for (int i = 0; i < volumeCount; ++i)
	{
		auto& volume = data["volumes"][i];
		const int vertCount = volume["verts"].size();
		if (vertCount > MAX_CONVEXVOL_PTS)
			continue;
		for (int j = 0; j < vertCount; ++j)
		{
			auto& jvert = volume["verts"][j];
			verts[j * 3] = jvert[0];
			verts[j * 3 + 1] = jvert[1];
			verts[j * 3 + 2] = jvert[2];
		}
		geom->addConvexVolume(volume["id"], verts, vertCount, info["hmin"], info["hmax"], (unsigned char)area, 0, nullptr);
	}
}