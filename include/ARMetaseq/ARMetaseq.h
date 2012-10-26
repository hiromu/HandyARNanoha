#ifndef __ARMetaseq_H__
#define __ARMetaseq_H__


/*=========================================================================
【設定項目】ユーザが任意で設定
=========================================================================*/
// libjpegの使用 (1:使用 0:未使用)
#define DEF_USE_LIBJPEG			0
// libpng の使用 (1:使用 0:未使用)
#define DEF_USE_LIBPNG			1
// 最大取り扱いテクスチャ
#define MAX_TEXTURE			100
// (1MQOファイル内の)最大オブジェクト数
#define MAX_OBJECT			50
// 文字列バッファのサイズ
#define SIZE_STR			256
// 1個のファイル内にあるオブジェクト(パーツ)の最大数
#define N_OBJ				MAX_OBJECT
// byte order指定(intel系:1)
#define DEF_IS_LITTLE_ENDIAN		1


/*=========================================================================
【設定】インクルードするヘッダ
=========================================================================*/
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
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


/*=========================================================================
【設定】画像の読み込み設定
=========================================================================*/
#ifdef D_JPEG
#undef DEF_USE_LIBJPEG
#define DEF_USE_LIBJPEG	1
#endif
#ifdef D_NO_JPEG
#undef DEF_USE_LIBJPEG
#define DEF_USE_LIBJPEG	0
#endif
#ifdef D_PNG
#undef DEF_USE_LIBPNG
#define DEF_USE_LIBPNG	1
#endif
#ifdef D_NO_PNG
#undef DEF_USE_LIBPNG
#define DEF_USE_LIBPNG	0
#endif


/*=========================================================================
【設定】 TGAの読み込み設定
=========================================================================*/
#define DEF_TGA_COLOR_MAP_FLAG_VALID 1
#define DEF_TGA_TYPE_NON 				0
#define DEF_TGA_TYPE_INDEX				1
#define DEF_TGA_TYPE_FULL				2
#define DEF_TGA_TYPE_MONO				3
#define DEF_TGA_TYPE_RLEINDEX			9
#define DEF_TGA_TYPE_RLEFULL			10
#define DEF_TGA_TYPE_RLEMONO			11
#define DEF_TGA_BIT_INFO_RIGHT_TO_LEFT	0x00
#define DEF_TGA_BIT_INFO_LEFT_TO_RIGHT	0x10
#define DEF_TGA_BIT_INFO_DOWN_TO_TOP	0x00
#define DEF_TGA_BIT_INFO_TOP_TO_DOWN	0x20


/*=========================================================================
【設定】 JPEGの読み込み設定
=========================================================================*/
#if DEF_USE_LIBJPEG
#define XMD_H
#ifdef FAR
#undef FAR
#endif
#include "jpeglib.h"
#ifdef WIN32
#pragma comment(lib,"libjpeg.lib")
#endif
#endif


/*=========================================================================
【設定】 PNGの読み込み設定
=========================================================================*/
#if DEF_USE_LIBPNG
#include "png.h"
#include "zlib.h"
#ifdef WIN32
#pragma comment(lib,"libpng.lib")
#pragma comment(lib,"zlib.lib")
#endif
#endif


/*=========================================================================
【設定】 VBOに関する設定
=========================================================================*/
#ifdef WIN32
#define GL_ARRAY_BUFFER_ARB 0x8892
#define GL_STATIC_DRAW_ARB 0x88E4
typedef void (APIENTRY * PFNGLBINDBUFFERARBPROC)
						(GLenum target, GLuint buffer);
typedef void (APIENTRY * PFNGLDELETEBUFFERSARBPROC)
					(GLsizei n, const GLuint *buffers);
typedef void (APIENTRY * PFNGLGENBUFFERSARBPROC)
						(GLsizei n, GLuint *buffers);
typedef void (APIENTRY * PFNGLBUFFERDATAARBPROC)
		(GLenum target, int size, const GLvoid *data, GLenum usage);
#endif

#ifdef __ARMETASEQ_C__
#define __ARMETASEQ_C__EXTERN
#else
#define __ARMETASEQ_C__EXTERN extern
#endif

// OpenGL頂点バッファサポートの有無
__ARMETASEQ_C__EXTERN int g_isVBOSupported;
#ifdef WIN32
// VBO Extension 関数のポインタ
__ARMETASEQ_C__EXTERN PFNGLGENBUFFERSARBPROC glGenBuffersARB;
__ARMETASEQ_C__EXTERN PFNGLBINDBUFFERARBPROC glBindBufferARB;
__ARMETASEQ_C__EXTERN PFNGLBUFFERDATAARBPROC glBufferDataARB;
__ARMETASEQ_C__EXTERN PFNGLDELETEBUFFERSARBPROC glDeleteBuffersARB;
#endif
#undef __ARMETASEQ_C__EXTERN


/*=========================================================================
【設定】 その他設定&マクロ
=========================================================================*/
#ifdef WIN32
#include <windows.h>
#else
#ifndef MAX_PATH
#define MAX_PATH 256
#endif

#ifndef TRUE
#define TRUE	(1==1)
#endif
#ifndef FALSE
#define FALSE	(1!=1)
#endif
#endif

#ifndef MAX
	#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif


/*=========================================================================
【型定義】 TGAのヘッダー用構造体(4色float)
=========================================================================*/
typedef struct {
	unsigned char id ;
	unsigned char color_map_flag ;
	unsigned char type ;
	unsigned short color_map_entry ;
	unsigned char color_map_entry_size ;
	unsigned short x ;
	unsigned short y ;
	unsigned short width ;
	unsigned short height ;
	unsigned char depth ;
	unsigned char bit_info ;
} STR_TGA_HEAD ;


/*=========================================================================
【型定義】 OpenGL用色構造体(4色float)
=========================================================================*/
#ifndef DEFINE_glCOLOR4f
#define DEFINE_glCOLOR4f
typedef struct tag_glCOLOR4f{
	GLfloat r;
	GLfloat g;
	GLfloat b;
	GLfloat a;
} glCOLOR4f;
#endif


/*=========================================================================
【型定義】 OpenGL用2次元座標構造体(float)
=========================================================================*/
#ifndef DEFINE_glPOINT2f
#define DEFINE_glPOINT2f
typedef struct tag_glPOINT2f {
	GLfloat x;
	GLfloat y;
} glPOINT2f;
#endif


/*=========================================================================
【型定義】 OpenGL用3次元座標構造体(float)
=========================================================================*/
#ifndef DEFINE_glPOINT3f
#define DEFINE_glPOINT3f
typedef struct tag_glPOINT3f{
	GLfloat x;
	GLfloat y;
	GLfloat z;
} glPOINT3f;
#endif


/*=========================================================================
【型定義】 面情報構造体
=========================================================================*/
#ifndef DEFINE_MQO_FACE
#define DEFINE_MQO_FACE
typedef struct tagMQO_FACE{
	int n;				// 1つの面を構成する頂点の数
	int m;				// 面の材質番号
	int v[4];			// 頂点番号を格納した配列
	glPOINT2f uv[4];		// UVマップ
} MQO_FACE;
#endif


/*=========================================================================
【型定義】 材質情報構造体
=========================================================================*/

#ifndef DEFINE_MQO_MATERIAL
#define DEFINE_MQO_MATERIAL
typedef struct tagMQO_MATERIAL{
	glCOLOR4f col;			// 色
	GLfloat dif[4];			// 拡散光
	GLfloat amb[4];			// 周囲光
	GLfloat emi[4];			// 自己照明
	GLfloat spc[4];			// 反射光
	GLfloat power;			// 反射光の強さ
	int useTex;			// テクスチャの有無
	char texFile[SIZE_STR];		// テクスチャファイル
	char alpFile[SIZE_STR];		// アルファテクスチャファイル
	GLuint texName;			// テクスチャ名
//	GLubyte *texImage;		// テクスチャ画像
} MQO_MATERIAL;
#endif


/*=========================================================================
【型定義】 オブジェクト構造体(パーツ1個のデータ)
=========================================================================*/
typedef struct tagMQO_OBJECT_V2 {
	char objname[SIZE_STR];		// パーツ名
	int visible;			// 可視
	float facet ;			//スムージング角
	int n_face;			// 面数
	int n_vertex;			// 頂点数
	MQO_FACE *F;			// 面配列
	glPOINT3f *V;			// 頂点配列
} MQO_OBJECT_V2 ;


/*=========================================================================
【型定義】 テクスチャ管理構造体
=========================================================================*/
typedef struct {
	GLuint texture_id ;
	int texsize ;
	char texfile[MAX_PATH];
	char alpfile[MAX_PATH];
	unsigned char alpha;
//	GLubyte *image;
} STR_TEXTURE_POOL;


/*=========================================================================
【型定義】 テクスチャ使用時の頂点配列の構造体
=========================================================================*/
#define DEF_MQO_STRINGLENGTH 64
typedef struct {
	GLfloat point[3];	// 頂点配列(x,y,z)
	GLfloat normal[3];	// 法線配列(x,y,z)
	GLfloat uv[2];		// UV配列(u, v)
} STR_MATERIAL_TEX_USE;


/*=========================================================================
【型定義】 ポリゴンのみの時の頂点配列の構造体
=========================================================================*/
typedef struct {
	GLfloat point[3];	// 頂点配列(x,y,z)
	GLfloat normal[3];	// 法線配列(x,y,z)
//	GLfloat color[4];
} STR_MATERIAL_ONLY;


/*=========================================================================
【型定義】 マテリアルごとの頂点配列の構造体
=========================================================================*/
typedef struct {
	// テクスチャの使用
	int isUseTexture;		// USE_TEXTURE/NOUSE_TEXTURE
	GLuint texture_id;		// テクスチャの名前(OpenGL)
	// 頂点バッファのID(OpenGL)
	GLuint VBO_id;			// 対応してる時だけ使用
	int datanum;			// 頂点数
	STR_MATERIAL_TEX_USE *texdata ;	// テクスチャ使用時の頂点配列
	GLfloat mat_color[4];		// 色配列(r,g,b,a)
	int isValidMaterialInfo;
	GLfloat dif[4];			// 拡散光
	GLfloat amb[4];			// 周囲光
	GLfloat emi[4];			// 自己照明
	GLfloat spc[4];			// 反射光
	GLfloat power;			// 反射光の強さ
	STR_MATERIAL_ONLY *matdata ;	// ポリゴンのみの時の頂点配列
} STR_MATERIAL;


/*=========================================================================
【型定義】 内部オブジェクトの配列の構造体
=========================================================================*/
typedef struct {
	char objname[DEF_MQO_STRINGLENGTH];	// オブジェクト名
	int isVisible;				// 0:非表示 その他:表示
	int matnum;				// 使用マテリアル数
	STR_MATERIAL *mat;			// マテリアル配列
} STR_MQO_INNER_OBJECT;

typedef struct {
	// 頂点配列作成時の指定アルファ値
	unsigned char alpha;		
	int objnum ;				// 内部オブジェクト数
	STR_MQO_INNER_OBJECT obj[MAX_OBJECT];	// 内部オブジェクト配列
} STR_MQO_OBJECT ;

// 独自形式へのアドレス
typedef STR_MQO_OBJECT *MQO_MODEL;
typedef STR_MQO_OBJECT *MQO_OBJECT;


/*=========================================================================
【型定義】 MQOシーケンス
=========================================================================*/
#ifndef DEFINE_MQO_SEQUENCE
#define DEFINE_MQO_SEQUENCE
typedef struct tagMQO_SEQUENCE {
	MQO_MODEL model;		// モデル
	int n_frame;			// フレーム数
} MQO_SEQUENCE;
#endif

typedef enum {
	E_MQO_VISIBLE,			// オブジェクト名／表示・非表示情報
	E_MQO_ALPHA,			// アルファ情報
	E_MQO_MAX
} E_MQO ;


/*=========================================================================
【プロトタイプ宣言】
=========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

GLubyte* LoadTexture(char *filename, int *tex_size, int isAlpha,
						unsigned char R, unsigned char G, unsigned char B);
void GLMetaseqInitialize();
void GLMetaseqClear();

MQO_OBJECT mqoCreateList(int num);
MQO_MODEL mqoCreateModel(char *filename, double scale);
MQO_SEQUENCE mqoCreateSequence(char *basename, int n_file, double scale);
int mqoCreateListObject(MQO_OBJECT displist, int num, char *filename,
										double scale, unsigned char alpha);

void mqoCallListObject(MQO_OBJECT object, int num);
void mqoCallModel(MQO_MODEL model);
void mqoCallSequence(MQO_SEQUENCE seq, int i);
void mqoClearObject(MQO_OBJECT object, int from, int num);
MQO_SEQUENCE mqoCreateSequenceEx(const char *basename, const char *format,
			int n_file, double scale, int fade_inout, unsigned char alpha);

void mqoDeleteModel(MQO_MODEL model);
void mqoDeleteObject(MQO_OBJECT object, int num);
void mqoDeleteSequence(MQO_SEQUENCE seq);


void mqoSet(E_MQO mode,MQO_MODEL object,...);
void mqoGet(E_MQO mode,MQO_MODEL object,...);
#ifdef __cplusplus
}
#endif


#endif
