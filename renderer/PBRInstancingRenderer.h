#ifndef PBR_INSTANCING_RENDERER_H
#define PBR_INSTANCING_RENDERER_H

#include "../bullet/examples/OpenGLWindow/GLInstancingRenderer.h"

/// GUI renderer for pybullet_pbr.
///
/// Reuses everything GLInstancingRenderer already does (shape registration,
/// instance buffers, transform sync, cameras, debug drawing) and replaces only
/// the main colour pass. Render modes we do not own are handed back to the base
/// class so getCameraImage()/segmentation/projective texture keep working.
class PBRInstancingRenderer : public GLInstancingRenderer
{
	struct PBRRendererData* m_pbr;

	bool loadShaders();
	void drawInstances();
	void bindShapeTexture(struct b3GraphicsInstance* gfxObj);
	void bindShapeMaterialTextures(int shapeIndex);
	unsigned int resolveTexture(int handle);
	void initSsaoKernel();
	void renderShadowMap();
	void renderLightShadows();
	bool renderBloom(const int viewport[4]);
	void renderLightShadowTile(const float viewProjection[16], const float lightPosition[3],
							   float range, int tile);
	void uploadLightUniforms();
	void updateEnvironment();
	void updatePreviousTransforms(int totalNumInstances);
	void recordCurrentTransforms(int totalNumInstances);
	int totalInstanceCount() const;

	void uploadMaterialTable();
	unsigned int loadEquirect(const char* path);
	void renderSsao(const float projectionMatrix[16], const float viewMatrix[16]);

public:
	PBRInstancingRenderer(int maxObjectCapacity, int maxShapeCapacityInBytes = 56 * 1024 * 1024);
	virtual ~PBRInstancingRenderer();

	virtual void renderSceneInternal(int orgRenderMode = B3_DEFAULT_RENDERMODE);
	virtual void removeAllInstances();

	/// PyBullet routes changeVisualShape(specularColor=...) here with the *graphics
	/// instance uid*, which is the per-(body, link, shape) key nothing else exposes.
	/// pybullet_pbr.set_material() rides on that to store a real per-instance material.
	virtual void writeSingleInstanceSpecularColorToCPU(const float* specular, int srcIndex2);
	virtual void writeSingleInstanceSpecularColorToCPU(const double* specular, int srcIndex2);

	/// Stores one instance's material. Public so the by-body entry point, used by the
	/// headless path, can reach it.
	void recordInstanceMaterial(int instanceUid, float metallic, float roughness, float marker);
};

#endif  // PBR_INSTANCING_RENDERER_H
