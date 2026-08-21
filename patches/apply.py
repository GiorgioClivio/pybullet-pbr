#!/usr/bin/env python3
"""Applies the (small, idempotent) integration patch to the local Bullet checkout.

Three changes only:
  1. move b3GraphicsInstance / InternalDataRenderer out of GLInstancingRenderer.cpp
     into a header, so a subclass can see them;
  2. make GLInstancingRenderer's data members protected;
  3. let SimpleOpenGL3App create our renderer instead of GLInstancingRenderer.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GLW = os.path.join(ROOT, "bullet", "examples", "OpenGLWindow")
HDR_NAME = "GLInstancingRendererInternalStructs.h"

HDR_PREAMBLE = """// Extracted verbatim from GLInstancingRenderer.cpp by pybullet_pbr's patches/apply.py,
// so that a GLInstancingRenderer subclass can see the renderer's internal state.
#ifndef GL_INSTANCING_RENDERER_INTERNAL_STRUCTS_H
#define GL_INSTANCING_RENDERER_INTERNAL_STRUCTS_H

#include "OpenGLInclude.h"
#include "SimpleCamera.h"
#include "GLRenderToTexture.h"
#include "GLInstanceRendererInternalData.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3ResizablePool.h"

"""


def cut(text, start_marker, end_marker):
    i = text.index(start_marker)
    j = text.index(end_marker, i)
    return text[:i], text[i:j], text[j:]


def patch_cpp():
    path = os.path.join(GLW, "GLInstancingRenderer.cpp")
    src = open(path).read()
    if HDR_NAME in src:
        return "already patched"

    pre, gfx, rest = cut(src, "struct b3GraphicsInstance\n{", "\nbool m_ortho = false;")
    rest_pre, internals, post = cut(rest, "struct InternalTextureHandle\n{",
                                    "struct GLInstanceRendererInternalData* GLInstancingRenderer::getInternalData()")

    open(os.path.join(GLW, HDR_NAME), "w").write(
        HDR_PREAMBLE + gfx.rstrip() + "\n\n" + internals.rstrip() + "\n\n#endif\n")
    open(path, "w").write(pre + '#include "%s"\n' % HDR_NAME + rest_pre + post)
    return "moved structs into " + HDR_NAME


def patch_header():
    path = os.path.join(GLW, "GLInstancingRenderer.h")
    src = open(path).read()
    if "protected:  // pybullet_pbr" in src:
        return "already patched"
    anchor = "class GLInstancingRenderer : public CommonRenderInterface\n{\n"
    assert anchor in src, "GLInstancingRenderer class declaration not found"
    src = src.replace(
        anchor, anchor + "protected:  // pybullet_pbr: subclasses need the renderer state\n", 1)
    open(path, "w").write(src)
    return "members made protected"


def patch_app():
    path = os.path.join(GLW, "SimpleOpenGL3App.cpp")
    src = open(path).read()
    if "b3CreateInstancingRenderer" in src:
        return "already patched"
    hook = """
// pybullet_pbr: weak factory hook. When the PBR renderer is linked in it provides
// this symbol and takes over the GUI viewport; otherwise the stock renderer is used.
__attribute__((weak)) GLInstancingRenderer* b3CreateInstancingRenderer(int maxObjects, int maxShapeBytes)
{
	return 0;
}
"""
    decl = "SimpleOpenGL3App::SimpleOpenGL3App("
    src = src.replace(decl, hook.lstrip() + "\n" + decl, 1)
    old = "m_instancingRenderer = new GLInstancingRenderer(maxNumObjectCapacity, maxShapeCapacityInBytes);"
    new = ("m_instancingRenderer = b3CreateInstancingRenderer(maxNumObjectCapacity, maxShapeCapacityInBytes);\n"
           "\tif (!m_instancingRenderer)\n"
           "\t\tm_instancingRenderer = new GLInstancingRenderer(maxNumObjectCapacity, maxShapeCapacityInBytes);")
    assert old in src, "renderer construction site not found"
    src = src.replace(old, new, 1)
    open(path, "w").write(src)
    return "SimpleOpenGL3App uses b3CreateInstancingRenderer()"



def patch_egl():
    """Let the EGL (headless) renderer use our renderer too.

    The plugin already builds on GLInstancingRenderer, so it is the same one-line
    factory swap as the GUI. The declaration is weak so the file still links when our
    sources are not part of the build.
    """
    path = os.path.join(ROOT, "bullet", "examples", "SharedMemory", "plugins",
                        "eglPlugin", "eglRendererVisualShapeConverter.cpp")
    src = open(path).read()
    if "b3CreateInstancingRenderer" in src:
        return "already patched"

    old = ("\t\tm_instancingRenderer = new GLInstancingRenderer(maxNumObjectCapacity, "
           "maxShapeCapacityInBytes);")
    assert old in src, "EGL renderer construction site not found"
    new = ("\t\t// pybullet_pbr: same factory hook as the GUI renderer\n"
           "\t\tm_instancingRenderer = b3CreateInstancingRenderer\n"
           "\t\t\t\t\t\t\t   ? b3CreateInstancingRenderer(maxNumObjectCapacity,\n"
           "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t maxShapeCapacityInBytes)\n"
           "\t\t\t\t\t\t\t   : 0;\n"
           "\t\tif (!m_instancingRenderer)\n"
           "\t\t\tm_instancingRenderer = new GLInstancingRenderer("
           "maxNumObjectCapacity, maxShapeCapacityInBytes);")
    src = src.replace(old, new, 1)

    anchor = "struct EGLRendererObjectArray"
    assert anchor in src, "EGL anchor not found"
    src = src.replace(anchor,
                      "// pybullet_pbr: provided when our renderer is linked in, null otherwise\n"
                      "__attribute__((weak)) GLInstancingRenderer* b3CreateInstancingRenderer(\n"
                      "\tint maxObjects, int maxShapeBytes);\n"
                      "// Reports which graphics instance belongs to which body and link. The\n"
                      "// server never forwards specular colours to a plugin renderer, so this is\n"
                      "// how per-instance materials reach the headless path.\n"
                      "extern \"C\" __attribute__((weak)) void pbr_register_instance(\n"
                      "\tint bodyUniqueId, int linkIndex, int graphicsUid);\n\n" + anchor, 1)

    # both places the converter creates an instance
    count = 0
    for old, body, link in (
            ("\t\tint graphicsIndex = m_data->m_instancingRenderer->registerGraphicsInstance(\n",
             "bodyUniqueId", "linkIndex"),
            ("\t\t\t\tint graphicsIndex = m_data->m_instancingRenderer->"
             "registerGraphicsInstance(shapeIndex, &visualShape.m_localVisualFrame[0], "
             "&visualShape.m_localVisualFrame[3], &visualShape.m_rgbaColor[0], scaling);\n",
             "visualShape.m_objectUniqueId", "visualShape.m_linkIndex")):
        if old not in src:
            continue
        # insert the report right after the statement the instance is created in
        start = src.index(old)
        end = src.index(";\n", start + len(old) - 2) + 2 if old.endswith("(\n") else start + len(old)
        report = ("\t\tif (pbr_register_instance)\n"
                  "\t\t\tpbr_register_instance(%s, %s, graphicsIndex);\n" % (body, link))
        src = src[:end] + report + src[end:]
        count += 1
    assert count, "no EGL instance registration site found"
    
    open(path, "w").write(src)
    return "EGL renderer uses b3CreateInstancingRenderer()"


def patch_setup():
    """Compile renderer/*.cpp into the pybullet extension and bake in the shader dir."""
    path = os.path.join(ROOT, "bullet", "setup.py")
    lines = open(path).read().split("\n")
    if any("PBRInstancingRenderer" in l for l in lines):
        return "already patched"

    out = []
    for line in lines:
        if line.startswith("pybullet_ext = Extension("):
            out.append("PBR_FLAGS = ['-DPBR_SHADER_DIR=\"%s\"' % os.path.abspath(")
            out.append("    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'renderer', 'shaders'))]")
            out.append("")
        out.append(line)
        if line.startswith('sources = ["examples/pybullet/pybullet.c"]'):
            out.append('+["../renderer/PBRInstancingRenderer.cpp"]' + chr(92))
        if line.startswith("egl_renderer_sources = "):
            out.append('["../renderer/PBRInstancingRenderer.cpp"]+' + chr(92))
    src = "\n".join(out)

    # only the pybullet extension gets the shader dir (eglRenderer keeps stock flags)
    old = "    extra_compile_args=CXX_FLAGS.split(),"
    assert old in src, "extra_compile_args not found"
    src = src.replace(old, "    extra_compile_args=CXX_FLAGS.split() + PBR_FLAGS,", 1)
    # the headless renderer needs the same shader path
    old_egl = "      extra_compile_args=(CXX_FLAGS + EGL_CXX_FLAGS).split(),"
    assert old_egl in src, "egl extra_compile_args not found"
    src = src.replace(old_egl,
                      "      extra_compile_args=(CXX_FLAGS + EGL_CXX_FLAGS).split() + PBR_FLAGS,", 1)
    open(path, "w").write(src)
    return "renderer sources + PBR_SHADER_DIR added"


if __name__ == "__main__":
    for fn in (patch_cpp, patch_header, patch_app, patch_egl, patch_setup):
        print("[patch] %s: %s" % (fn.__name__, fn()))
