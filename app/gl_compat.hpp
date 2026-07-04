// GL 1.1 comes from <GL/gl.h> (pulled in by GLFW). The extension enums and
// PFNGL* typedefs the app uses come from <GL/glext.h> where it exists;
// Windows' SDK stops at GL 1.1 and ships no glext.h, so define the handful
// we actually use ourselves. The functions themselves are always loaded at
// runtime via glfwGetProcAddress (see WEFT_GL_FUNCS in main.cpp).
#pragma once

#if defined(__has_include) && __has_include(<GL/glext.h>)
#include <GL/glext.h>
#else

#include <cstddef>

#ifndef APIENTRY
#define APIENTRY
#endif
#ifndef APIENTRYP
#define APIENTRYP APIENTRY *
#endif

typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

#define GL_ARRAY_BUFFER 0x8892
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82

typedef GLuint(APIENTRYP PFNGLCREATESHADERPROC)(GLenum type);
typedef void(APIENTRYP PFNGLSHADERSOURCEPROC)(GLuint shader, GLsizei count,
                                              const GLchar* const* string,
                                              const GLint* length);
typedef void(APIENTRYP PFNGLCOMPILESHADERPROC)(GLuint shader);
typedef void(APIENTRYP PFNGLGETSHADERIVPROC)(GLuint shader, GLenum pname,
                                             GLint* params);
typedef void(APIENTRYP PFNGLGETSHADERINFOLOGPROC)(GLuint shader,
                                                  GLsizei bufSize,
                                                  GLsizei* length,
                                                  GLchar* infoLog);
typedef GLuint(APIENTRYP PFNGLCREATEPROGRAMPROC)(void);
typedef void(APIENTRYP PFNGLATTACHSHADERPROC)(GLuint program, GLuint shader);
typedef void(APIENTRYP PFNGLLINKPROGRAMPROC)(GLuint program);
typedef void(APIENTRYP PFNGLGETPROGRAMIVPROC)(GLuint program, GLenum pname,
                                              GLint* params);
typedef void(APIENTRYP PFNGLGETPROGRAMINFOLOGPROC)(GLuint program,
                                                   GLsizei bufSize,
                                                   GLsizei* length,
                                                   GLchar* infoLog);
typedef void(APIENTRYP PFNGLDELETESHADERPROC)(GLuint shader);
typedef void(APIENTRYP PFNGLUSEPROGRAMPROC)(GLuint program);
typedef GLint(APIENTRYP PFNGLGETUNIFORMLOCATIONPROC)(GLuint program,
                                                     const GLchar* name);
typedef void(APIENTRYP PFNGLUNIFORMMATRIX4FVPROC)(GLint location,
                                                  GLsizei count,
                                                  GLboolean transpose,
                                                  const GLfloat* value);
typedef void(APIENTRYP PFNGLUNIFORM1FPROC)(GLint location, GLfloat v0);
typedef void(APIENTRYP PFNGLUNIFORM1IPROC)(GLint location, GLint v0);
typedef void(APIENTRYP PFNGLUNIFORM3FVPROC)(GLint location, GLsizei count,
                                            const GLfloat* value);
typedef void(APIENTRYP PFNGLGENVERTEXARRAYSPROC)(GLsizei n, GLuint* arrays);
typedef void(APIENTRYP PFNGLBINDVERTEXARRAYPROC)(GLuint array);
typedef void(APIENTRYP PFNGLGENBUFFERSPROC)(GLsizei n, GLuint* buffers);
typedef void(APIENTRYP PFNGLBINDBUFFERPROC)(GLenum target, GLuint buffer);
typedef void(APIENTRYP PFNGLBUFFERDATAPROC)(GLenum target, GLsizeiptr size,
                                            const void* data, GLenum usage);
typedef void(APIENTRYP PFNGLVERTEXATTRIBPOINTERPROC)(
    GLuint index, GLint size, GLenum type, GLboolean normalized,
    GLsizei stride, const void* pointer);
typedef void(APIENTRYP PFNGLENABLEVERTEXATTRIBARRAYPROC)(GLuint index);

#endif  // no <GL/glext.h>
