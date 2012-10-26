#ifdef WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
	#include <OpenGL/gl.h>
	#include <OpenGL/glu.h>
	#include <GLUT/glut.h>
	#include <gl/glext.h>
#else
	#include <GL/gl.h>
	#include <GL/glu.h>
	#include <GL/glut.h>
	#include <GL/glext.h>
#endif


int IsExtensionSupported( char* szTargetExtension ) ;
