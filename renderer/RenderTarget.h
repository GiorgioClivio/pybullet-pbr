#ifndef PBR_RENDER_TARGET_H
#define PBR_RENDER_TARGET_H

#include <stdio.h>
#include <stdlib.h>

#include "../bullet/examples/OpenGLWindow/OpenGLInclude.h"

/// One off-screen colour attachment, optionally with a depth texture.
///
/// The depth attachment is a texture rather than a renderbuffer so screen-space
/// passes can sample it; it stays blit-compatible with the default framebuffer,
/// which is how the debug/UI overlay keeps depth-testing against the scene.
class RenderTarget
{
	GLuint m_fbo;
	GLuint m_color;
	GLuint m_color2;
	GLuint m_depth;
	GLenum m_format;
	GLenum m_format2;
	bool m_withDepth;
	int m_width;
	int m_height;

	void destroy()
	{
		if (m_color) glDeleteTextures(1, &m_color);
		if (m_color2) glDeleteTextures(1, &m_color2);
		if (m_depth) glDeleteTextures(1, &m_depth);
		if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
		m_fbo = m_color = m_color2 = m_depth = 0;
		m_width = m_height = 0;
	}

public:
	RenderTarget() : m_fbo(0), m_color(0), m_color2(0), m_depth(0), m_format(0), m_format2(0),
					 m_withDepth(false), m_width(0), m_height(0) {}
	~RenderTarget() { destroy(); }

	GLuint colorTexture() const { return m_color; }

	/// Builds the mip chain of the colour attachment. The smallest level is the frame's
	/// average, which is how auto-exposure meters it, and the middle levels are a
	/// ready-made blur for bloom - both for the cost of one call.
	void generateColorMips()
	{
		if (!m_color)
			return;
		glBindTexture(GL_TEXTURE_2D, m_color);
		// Allocate and fill the levels, but leave GL_TEXTURE_MIN_FILTER alone. Setting
		// it to a mipmap filter would make every later plain texture() read of this
		// attachment depend on the mip chain still being current - and after a resize
		// it is not, so those reads come back black.
		glGenerateMipmap(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	GLuint secondColorTexture() const { return m_color2; }
	GLuint depthTexture() const { return m_depth; }
	GLuint fbo() const { return m_fbo; }
	int width() const { return m_width; }
	int height() const { return m_height; }

	/// (Re)allocates when the size or format changed. False means the FBO is incomplete.
	/// `internalFormat` of 0 makes a depth-only target set up for shadow comparison
	/// sampling (sampler2DShadow). A non-zero `secondFormat` adds COLOR_ATTACHMENT1,
	/// for a pass that writes two outputs (colour and motion vectors).
	bool resize(int width, int height, GLenum internalFormat, bool withDepth,
				GLenum secondFormat = 0)
	{
		if (width <= 0 || height <= 0) return false;
		if (m_fbo && width == m_width && height == m_height &&
			internalFormat == m_format && withDepth == m_withDepth &&
			secondFormat == m_format2)
			return true;
		destroy();
		m_format = internalFormat;
		m_format2 = secondFormat;
		m_withDepth = withDepth;

		if (internalFormat == 0)
		{
			// depth-only: a shadow map
			glGenTextures(1, &m_depth);
			glBindTexture(GL_TEXTURE_2D, m_depth);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
						 GL_DEPTH_COMPONENT, GL_FLOAT, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			float border[4] = {1.f, 1.f, 1.f, 1.f};  // outside the map = fully lit
			glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
			// No GL_TEXTURE_COMPARE_MODE: the shader reads raw depths so it can search
			// for blockers and size the penumbra, which hardware comparison hides.
			glBindTexture(GL_TEXTURE_2D, 0);

			glGenFramebuffers(1, &m_fbo);
			glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depth, 0);
			glDrawBuffer(GL_NONE);
			glReadBuffer(GL_NONE);
			GLenum depthStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			if (depthStatus != GL_FRAMEBUFFER_COMPLETE)
			{
				destroy();
				return false;
			}
			m_width = width;
			m_height = height;
			return true;
		}

		GLenum type = (internalFormat == GL_RGBA16F || internalFormat == GL_R16F)
						  ? GL_FLOAT : GL_UNSIGNED_BYTE;
		GLenum channels = (internalFormat == GL_R8 || internalFormat == GL_R16F) ? GL_RED : GL_RGBA;

		glGenTextures(1, &m_color);
		glBindTexture(GL_TEXTURE_2D, m_color);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, channels, type, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		if (withDepth)
		{
			glGenTextures(1, &m_depth);
			glBindTexture(GL_TEXTURE_2D, m_depth);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
						 GL_DEPTH_COMPONENT, GL_FLOAT, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		}
		glBindTexture(GL_TEXTURE_2D, 0);

		if (secondFormat)
		{
			GLenum type2 = (secondFormat == GL_RG16F || secondFormat == GL_RGBA16F)
							   ? GL_FLOAT : GL_UNSIGNED_BYTE;
			GLenum channels2 = (secondFormat == GL_RG16F) ? GL_RG : GL_RGBA;
			glGenTextures(1, &m_color2);
			glBindTexture(GL_TEXTURE_2D, m_color2);
			glTexImage2D(GL_TEXTURE_2D, 0, secondFormat, width, height, 0, channels2, type2, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glBindTexture(GL_TEXTURE_2D, 0);
		}

		glGenFramebuffers(1, &m_fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_color, 0);
		if (secondFormat)
		{
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_color2, 0);
			GLenum buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
			glDrawBuffers(2, buffers);
		}
		if (withDepth)
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depth, 0);
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		if (status != GL_FRAMEBUFFER_COMPLETE)
		{
			destroy();
			return false;
		}
		m_width = width;
		m_height = height;
		return true;
	}
};

#endif  // PBR_RENDER_TARGET_H
