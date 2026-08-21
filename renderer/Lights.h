#ifndef PBR_LIGHTS_H
#define PBR_LIGHTS_H

/// Movable point and spot lights.
///
/// Shadow maps for these go into one shared atlas of square tiles rather than a
/// texture array, which keeps the shaders at GLSL 330 and costs one texture unit.
/// The atlas stores *distance to the light*, normalised by the light's range, not
/// post-projection depth: a point light then needs no matrices in the fragment
/// shader at all (cube-face selection is pure arithmetic on the light vector), and
/// the bias is in metres for both light types instead of nonlinear depth units.
#define PBR_MAX_LIGHTS 8
#define PBR_MAX_SHADOW_TILES 16

#define PBR_LIGHT_OFF 0
#define PBR_LIGHT_POINT 1
#define PBR_LIGHT_SPOT 2

struct PbrLight
{
	int m_type;
	float m_position[3];
	float m_direction[3];   // spot only, points away from the light
	float m_color[3];
	float m_intensity;
	float m_range;          // metres; light is clamped to zero beyond it
	float m_innerDegrees;   // spot cone, full angle
	float m_outerDegrees;
	float m_radius;         // emitter size in metres: soft shadows and highlight size
	int m_castsShadow;
	int m_shadowTile;       // first tile in the atlas, -1 when it has none

	PbrLight()
		: m_type(PBR_LIGHT_OFF), m_intensity(0.f), m_range(10.f),
		  m_innerDegrees(25.f), m_outerDegrees(40.f), m_radius(0.f), m_castsShadow(1),
		  m_shadowTile(-1)
	{
		m_position[0] = m_position[1] = m_position[2] = 0.f;
		m_direction[0] = 0.f;
		m_direction[1] = 0.f;
		m_direction[2] = -1.f;
		m_color[0] = m_color[1] = m_color[2] = 1.f;
	}
};

#endif  // PBR_LIGHTS_H
