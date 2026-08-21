#ifndef PBR_ENVIRONMENT_H
#define PBR_ENVIRONMENT_H

#include <math.h>
#include <stdio.h>
#include <string>

#include "../bullet/examples/OpenGLWindow/OpenGLInclude.h"
#include "Bullet3Common/b3Logging.h"

/// Image-based lighting: an environment cubemap plus the two convolutions and the
/// BRDF lookup table that Cook-Torrance needs to shade from it.
///
/// Everything here is built once, when the environment changes - never per frame.
/// The source is either an equirectangular HDR file or a procedural sky; both go
/// through the same capture shader, so the runtime path has one case, not two.
class Environment
{
	GLuint m_cubemap;
	GLuint m_irradiance;
	GLuint m_prefiltered;
	GLuint m_brdfLut;
	GLuint m_fbo;
	bool m_ready;
	std::string m_source;

	static const int CUBE_SIZE = 256;
	static const int IRRADIANCE_SIZE = 32;
	static const int PREFILTER_SIZE = 128;
	static const int PREFILTER_MIPS = 5;
	static const int BRDF_SIZE = 256;

	static GLuint makeCubemap(int size, bool mipmapped)
	{
		GLuint tex = 0;
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_CUBE_MAP, tex);
		for (int f = 0; f < 6; f++)
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, 0, GL_RGB16F, size, size, 0,
						 GL_RGB, GL_FLOAT, 0);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,
						mipmapped ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
		if (mipmapped)
			glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
		return tex;
	}

public:
	Environment() : m_cubemap(0), m_irradiance(0), m_prefiltered(0), m_brdfLut(0),
					m_fbo(0), m_ready(false) {}

	~Environment() { destroy(); }

	void destroy()
	{
		if (m_cubemap) glDeleteTextures(1, &m_cubemap);
		if (m_irradiance) glDeleteTextures(1, &m_irradiance);
		if (m_prefiltered) glDeleteTextures(1, &m_prefiltered);
		if (m_brdfLut) glDeleteTextures(1, &m_brdfLut);
		if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
		m_cubemap = m_irradiance = m_prefiltered = m_brdfLut = m_fbo = 0;
		m_ready = false;
	}

	bool ready() const { return m_ready; }
	GLuint cubemap() const { return m_cubemap; }
	GLuint irradiance() const { return m_irradiance; }
	GLuint prefiltered() const { return m_prefiltered; }
	GLuint brdfLut() const { return m_brdfLut; }
	static int prefilterMips() { return PREFILTER_MIPS; }
	const std::string& source() const { return m_source; }

	/// Builds every IBL texture. `equirectTexture` of 0 means "use the procedural
	/// sky in the capture shader". `drawFace` renders a fullscreen triangle for the
	/// currently bound program and face uniform.
	template <class FaceDrawer>
	bool build(const std::string& source, GLuint equirectTexture, FaceDrawer& draw)
	{
		destroy();
		m_source = source;
		glGenFramebuffers(1, &m_fbo);
		m_cubemap = makeCubemap(CUBE_SIZE, true);
		m_irradiance = makeCubemap(IRRADIANCE_SIZE, false);
		m_prefiltered = makeCubemap(PREFILTER_SIZE, true);

		glGenTextures(1, &m_brdfLut);
		glBindTexture(GL_TEXTURE_2D, m_brdfLut);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, BRDF_SIZE, BRDF_SIZE, 0, GL_RG, GL_FLOAT, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0);

		glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

		// 1. capture the source into the environment cubemap
		draw.beginCapture(equirectTexture);
		for (int f = 0; f < 6; f++)
		{
			attach(m_cubemap, f, 0, CUBE_SIZE);
			draw.face(f);
		}
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_cubemap);
		glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

		// 2. diffuse irradiance
		draw.beginIrradiance(m_cubemap);
		for (int f = 0; f < 6; f++)
		{
			attach(m_irradiance, f, 0, IRRADIANCE_SIZE);
			draw.face(f);
		}

		// 3. prefiltered specular, one mip per roughness step
		draw.beginPrefilter(m_cubemap, (float)CUBE_SIZE);
		for (int mip = 0; mip < PREFILTER_MIPS; mip++)
		{
			int size = PREFILTER_SIZE >> mip;
			draw.setRoughness((float)mip / (float)(PREFILTER_MIPS - 1));
			for (int f = 0; f < 6; f++)
			{
				attach(m_prefiltered, f, mip, size);
				draw.face(f);
			}
		}

		// 4. the environment-independent BRDF table
		draw.beginBrdf();
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_brdfLut, 0);
		glViewport(0, 0, BRDF_SIZE, BRDF_SIZE);
		draw.face(0);

		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		if (status != GL_FRAMEBUFFER_COMPLETE)
		{
			b3Warning("pybullet_pbr: environment framebuffer incomplete (0x%x)", status);
			destroy();
			return false;
		}
		m_ready = true;
		return true;
	}

private:
	void attach(GLuint cubemap, int face, int mip, int size)
	{
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
							   GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, cubemap, mip);
		glViewport(0, 0, size, size);
	}
};

#endif  // PBR_ENVIRONMENT_H
