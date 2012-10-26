#define __ARMETASEQ_C__

#include <ARMetaseq/ARMetaseq.h>
#include "OpenGLExtension.h"

#define DEF_DEFAULT_FORMAT "%s%d.mqo"

////////////////////////////////////////////////////////////////////////////////////////////
// プロトタイプ宣言

void mqoGetDirectory(const char *path_file, char *path_dir);
GLubyte* mqoLoadTexture(char *filename,int *tex_size);
void mqoRegistTexture(GLuint* tex_name,GLubyte* tex_img,int tex_size);
void mqoReleaseTexture(void *pImage);
void mqoSnormal(glPOINT3f A, glPOINT3f B, glPOINT3f C, glPOINT3f *normal);
void mqoReadMaterial(FILE *fp, MQO_MATERIAL M[]);
void mqoReadVertex(FILE *fp, glPOINT3f V[]);
void mqoReadFace(FILE *fp, MQO_FACE F[]);
void mqoReadObject(FILE *fp, MQO_OBJECT_V2 *obj);
MQO_MODEL	 mqoCreateModel(char *filename, double scale);
MQO_SEQUENCE mqoCreateSequence(char *basename, int n_file, double scale);
void mqoCallModel(MQO_MODEL model);
void mqoCallSequence(MQO_SEQUENCE seq, int i);
void mqoDeleteModel(MQO_MODEL model);
void mqoDeleteSequence(MQO_SEQUENCE seq);


void mqoMakeObjectsEx(STR_MQO_OBJECT *mqoobj,MQO_OBJECT_V2 obj[], int n_obj, MQO_MATERIAL M[],int n_mat,
					  double scale,unsigned int alpha) ;
void mqoMakePolygonEx(char *objname,int isVisible,STR_MQO_OBJECT * mqoobj,
					  MQO_FACE F[],int fnum,glPOINT3f V[],glPOINT3f N[],MQO_MATERIAL M[],
					  int n_mat,double facet,double scale,unsigned char alpha) ;
int mqoReadBVertex(FILE *fp,glPOINT3f V[]) ;
GLubyte* mqoLoadTextureEx(char *texfile,char *alpfile,int *tex_size,unsigned char alpha) ;


//byte orderを変換する
void endianConverter(void *addr,unsigned int size) {
	unsigned int pos ;
	char c;
	if( size <= 1 ) return ;
	for( pos = 0 ; pos < size/2 ; pos++ ) {
		c = *(((char *)addr)+pos) ;
		*(((char *)addr)+pos) = *(((char *)addr)+(size-1 - pos)) ;
		*(((char *)addr)+(size-1 - pos)) = c ;
	}
}
void TGAHeaderEndianConverter(	STR_TGA_HEAD *tgah ) {
	endianConverter(&tgah->color_map_entry,sizeof(tgah->color_map_entry)) ;
	endianConverter(&tgah->x,sizeof(tgah->x)) ;
	endianConverter(&tgah->y,sizeof(tgah->y)) ;
	endianConverter(&tgah->width,sizeof(tgah->width)) ;
	endianConverter(&tgah->height,sizeof(tgah->height)) ;
}

int l_texPoolnum ;
STR_TEXTURE_POOL l_texPool[MAX_TEXTURE] ;
void mqoClearTexturePool() ;

int l_GLMetaseqInitialized = 0 ; // 初期化フラグ

//初期化処理
//argInit()の後に呼ばないとちゃんと動かない。
void GLMetaseqInitialize() {
	memset(l_texPool,0,sizeof(l_texPool)) ;
	l_texPoolnum = 0 ;

	g_isVBOSupported = 0 ;
#ifdef WIN32
	g_isVBOSupported = IsExtensionSupported("GL_ARB_vertex_buffer_object") ;
//	g_isVBOSupported = 0 ;
	glGenBuffersARB = NULL ;
	glBindBufferARB = NULL ;
	glBufferDataARB = NULL ;
	glDeleteBuffersARB = NULL ;

	if( g_isVBOSupported ) {
		printf("OpenGL : 頂点バッファをサポートしている→使用する\n") ;
		// GL 関数のポインタを所得する
		glGenBuffersARB = (PFNGLGENBUFFERSARBPROC) wglGetProcAddress("glGenBuffersARB");
		glBindBufferARB = (PFNGLBINDBUFFERARBPROC) wglGetProcAddress("glBindBufferARB");
		glBufferDataARB = (PFNGLBUFFERDATAARBPROC) wglGetProcAddress("glBufferDataARB");
		glDeleteBuffersARB = (PFNGLDELETEBUFFERSARBPROC) wglGetProcAddress("glDeleteBuffersARB");
	}
#endif
	l_GLMetaseqInitialized = 1 ;

}
//使用終了時の処理
void GLMetaseqClear() {
	mqoClearTexturePool() ;
}


//テクスチャプール
//テクスチャがまだ読み込まれていなければ読み込み、テクスチャ登録
//すでに読み込まれていれば登録したものを返す。
GLuint mqoSetTexturePool(char *texfile,char *alpfile,unsigned char alpha) 
{
	int pos ;
	GLubyte *image ;
	for( pos = 0 ; pos < l_texPoolnum ; pos++ ) {
		if( alpha != l_texPool[pos].alpha ) {
			continue ;
		}
		if( texfile != NULL ) {
			if( strcmp(texfile,l_texPool[pos].texfile) != 0 ) {
				continue ;
			}
		}
		if( alpfile != NULL ) {
			if( strcmp(alpfile,l_texPool[pos].alpfile) != 0 ) {
				continue ;
			}
		}
		break ;
	}
	if( pos < l_texPoolnum ) { //すでに読み込み済み
		return  l_texPool[pos].texture_id ;
	}
	if( MAX_TEXTURE <= pos ) {
		printf("%s:mqoSetTexturePool テクスチャ読み込み領域不足\n",__FILE__) ;
		return -1 ;
	}
	image = mqoLoadTextureEx(texfile,alpfile,&l_texPool[pos].texsize,alpha) ;
	if( image == NULL ) {
		return -1 ;
	}

	if( texfile != NULL ) strncpy(l_texPool[pos].texfile,texfile,MAX_PATH) ;
	if( alpfile != NULL ) strncpy(l_texPool[pos].alpfile,alpfile,MAX_PATH) ;
	l_texPool[pos].alpha = alpha ;

	glPixelStorei(GL_UNPACK_ALIGNMENT,4);
	glPixelStorei(GL_PACK_ALIGNMENT,4);
	glGenTextures(1,&l_texPool[pos].texture_id);		// テクスチャを生成
	glBindTexture(GL_TEXTURE_2D,l_texPool[pos].texture_id);	// テクスチャの割り当て
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, l_texPool[pos].texsize, l_texPool[pos].texsize,
					0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	l_texPoolnum = pos+1 ;
	//登録すれば、読み込んだバッファは不要
	free(image) ;

	return l_texPool[pos].texture_id;
}
//テクスチャプールの開放
void mqoClearTexturePool()
{
	int pos ;
	for( pos = 0 ; pos < l_texPoolnum ; pos++ ) {
		glDeleteTextures(1, &l_texPool[pos].texture_id);			// テクスチャ情報を削除
	}

	memset(l_texPool,0,sizeof(l_texPool)) ;
	l_texPoolnum = 0 ;

}
/*=========================================================================
【関数】mqoLoadTextureEx
【用途】ファイルからテクスチャ画像を作成する
【引数】
		*texfile	ファイル名
		*alpfile	アルファファイル名
		*tex_size	テクスチャのサイズ（一辺の長さ）を返す

【戻値】テクスチャ画像へのポインタ（失敗時はNULL）
【仕様】24bitビットマップ,8,24,32bitＴＧＡ
		サイズは「一辺が2のn乗の正方形」に限定
		libjpeg,libpng（外部ライブラリ）が有ればJPEG,PNGの読み込み可能
=========================================================================*/

GLubyte* mqoLoadTextureEx(char *texfile,char *alpfile,int *tex_size,unsigned char alpha)
{
	FILE *fp;
	size_t namelen ;
	char ext[4];
	char wbuf[3];
	int isTGA ;
	int isPNG ;
	int isJPEG ;
	int other ;
	int	y,x,size;
	int fl;
	char *filename[2] ;
	int width[2] ;
	int sts ;
	STR_TGA_HEAD tgah;
	GLubyte	*pImage, *pRead;

#if DEF_USE_LIBJPEG
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	JSAMPARRAY jpegimage;
#endif
#if DEF_USE_LIBPNG
	unsigned char **pngimage ;
	unsigned long   pngwidth, pngheight;
	int				pngdepth ;
	int             color_type ;
#endif

	filename[0] = texfile ;
	filename[1] = alpfile ;
	width[0] = -1 ;
	width[1] = -1 ;
	pImage = NULL ;
	fp = NULL ;
	sts = 0 ;
#if DEF_USE_LIBJPEG
	jpegimage = NULL ;
#endif
#if DEF_USE_LIBPNG
	pngimage = NULL ;
#endif
	size = - 1;
	for( fl = 0 ; fl < 2 ; fl++ ) {//テクスチャ＝fl=0    アルファ＝fl=1
		if( filename[fl] == NULL ) continue ;
		namelen = strlen(filename[fl]) ;
		ext[0] = tolower(filename[fl][namelen-3]) ;
		ext[1] = tolower(filename[fl][namelen-2]) ;
		ext[2] = tolower(filename[fl][namelen-1]) ;
		ext[3] = 0x00 ;
		isTGA = (strcmp(ext,"tga")==0)?1:0 ;
		isPNG = (strcmp(ext,"png")==0)?1:0 ;
		isJPEG = (strcmp(ext,"jpg")==0)?1:0 ;
		if( (! isTGA) && (! isPNG) &&(! isJPEG) ) {
			filename[fl][namelen-3] = 'b' ;
			filename[fl][namelen-2] = 'm' ;
			filename[fl][namelen-1] = 'p' ;
		}
		if( fl == 1 ) { //アルファの読み込みはＴＧＡorＰＮＧ
			if( ! (isTGA || isPNG) ) {
				printf("アルファのファイルに対応できない→%s\n",filename[fl]) ;
				break ;
			}
		}
		if( fp != NULL ) fclose(fp) ;
		if ( (fp=fopen(filename[fl],"rb"))==NULL ) {
			printf("%s:テクスチャ読み込みエラー[%s]\n",__FILE__,filename[fl]) ;
			continue ;
		}
		// ヘッダのロード
		if( isTGA ) {
			fread(&tgah,sizeof(STR_TGA_HEAD),1,fp);
#if DEF_IS_LITTLE_ENDIAN
#else
			TGAHeaderEndianConverter(&tgah) ;
#endif
			size = width[fl] = tgah.width ;
		}
		if( isJPEG ) {
#if DEF_USE_LIBJPEG
			unsigned int i ;
			cinfo.err = jpeg_std_error( &jerr );
			jpeg_create_decompress( &cinfo );	//解凍用情報作成
			jpeg_stdio_src( &cinfo, fp );		//読み込みファイル指定
			jpeg_read_header( &cinfo, TRUE );	//jpegヘッダ読み込み
			jpeg_start_decompress( &cinfo );	//解凍開始

			if( cinfo.out_color_components == 3 && cinfo.out_color_space == JCS_RGB ) {
				if( jpegimage != NULL ) {
					for (i = 0; i < cinfo.output_height; i++) free(jpegimage[i]);            // 以下２行は２次元配列を解放します
					free(jpegimage);
				}
				//読み込みデータ配列の作成
				jpegimage = (JSAMPARRAY)malloc( sizeof( JSAMPROW ) * cinfo.output_height );
				for ( i = 0; i < cinfo.output_height; i++ ) {
					jpegimage[i] = (JSAMPROW)malloc( sizeof( JSAMPLE ) * cinfo.out_color_components * cinfo.output_width );
				}
				//解凍データ読み込み
				while( cinfo.output_scanline < cinfo.output_height ) {
					jpeg_read_scanlines( &cinfo,
						jpegimage + cinfo.output_scanline,
						cinfo.output_height - cinfo.output_scanline
					);
				}
				size = width[fl] = cinfo.output_width ;
			}

			jpeg_finish_decompress( &cinfo );	//解凍終了
			jpeg_destroy_decompress( &cinfo );	//解凍用情報解放
			if( !(cinfo.out_color_components == 3 && cinfo.out_color_space == JCS_RGB) ) {
				printf("JPEG 対応できないフォーマット→%s\n",filename[fl]) ;
			}
#else
			printf("このテクスチャは対応できないフォーマット→%s\n",filename[fl]) ;
			continue ;
#endif
		}
		if( isPNG ) {
#if DEF_USE_LIBPNG
			png_structp     png_ptr;
			png_infop       info_ptr;
			int             bit_depth, interlace_type;
			unsigned int             i ;
			int j,k;
			png_ptr = png_create_read_struct(                       // png_ptr構造体を確保・初期化します
							PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
			info_ptr = png_create_info_struct(png_ptr);             // info_ptr構造体を確保・初期化します
			png_init_io(png_ptr, fp);                               // libpngにfpを知らせます
			png_read_info(png_ptr, info_ptr);                       // PNGファイルのヘッダを読み込みます
			png_get_IHDR(png_ptr, info_ptr, &pngwidth, &pngheight,        // IHDRチャンク情報を取得します
							&bit_depth, &color_type, &interlace_type,
							&j,&k);
			if( pngimage != NULL ) {
				for (i = 0; i < pngheight; i++) free(pngimage[i]);            // 以下２行は２次元配列を解放します
				free(pngimage);
			}
			pngimage = (png_bytepp)malloc(pngheight * sizeof(png_bytep)); // 以下３行は２次元配列を確保します
			i = png_get_rowbytes(png_ptr, info_ptr) ;
			pngdepth = i / pngwidth ;
			for (i = 0; i < pngheight; i++)
					pngimage[i] = (png_bytep)malloc(png_get_rowbytes(png_ptr, info_ptr));
			png_read_image(png_ptr, pngimage);                         // 画像データを読み込みます

			png_destroy_read_struct(                                // ２つの構造体のメモリを解放します
	        &png_ptr, &info_ptr, (png_infopp)NULL);
			size = width[fl] = pngwidth ;
#else
			printf("このテクスチャは対応できないフォーマット→%s\n",filename[fl]) ;
			continue ;
#endif
		}
		if( width[fl] == -1 ) {//ココまできてサイズが指定されていない　＝　ビットマップ
			fseek(fp,14+4,SEEK_SET);		// 画像幅が格納されている位置までシーク
			fread(&size,sizeof(int),1,fp);	// BiWidthの情報だけ取得
			fseek(fp,14+40,SEEK_SET);		// 画素データが格納されている位置までシーク
#if DEF_IS_LITTLE_ENDIAN
#else
			endianConverter(&size,sizeof(int)) ;
#endif
			width[fl] = size ;

		}
		if( width[0] != -1 && width[1] != -1 ) {
			if( width[0] != width[1] ) {
				sts = -1 ;
				break ;
			}
		}
		if( fl == 1 && isTGA ) { //アルファの読み込みはＴＧＡの８ビットモノクロｏｒ３２ビットフル
			if( !(
				(tgah.depth == 8 && tgah.type == DEF_TGA_TYPE_MONO) ||
				(tgah.depth == 32 && tgah.type == DEF_TGA_TYPE_FULL) 
				) ) {
				break ;
			}
		}
		if( fl == 1 && isPNG ) { //アルファの読み込みはＰＮＧのトゥルーカラー＋アルファｏｒグレースケール＋アルファ
			if( !(
				(color_type== 6 ) ||
				(color_type== 4 ) 
				) ) {
				break ;
			}
		}

		// メモリの確保
		if( pImage == NULL ) {
			pImage = (GLubyte*)malloc(sizeof(unsigned char)*size*size*4);
		}
		if (pImage==NULL) return NULL;
		for (y=0; y<size; y++){
			pRead = pImage + (size-1-y)*4*size;
			for (x=0; x<size; x++) {
				other = 1 ;
				if( fl == 0 ) {
					if( isJPEG ) {
#if DEF_USE_LIBJPEG
						pRead[0]= jpegimage[size-1-y][x*3] ;
						pRead[1]= jpegimage[size-1-y][x*3+1] ;
						pRead[2]= jpegimage[size-1-y][x*3+2] ;
						pRead[3] = alpha;				// A
						other = 0 ;
#endif
					}
					if( isPNG ) {
#if DEF_USE_LIBPNG
						if( color_type == 2 || color_type==6 ) { 
							pRead[0]= pngimage[size-1-y][x*pngdepth] ;
							pRead[1]= pngimage[size-1-y][x*pngdepth+1] ;
							pRead[2]= pngimage[size-1-y][x*pngdepth+2] ;
							pRead[3] = alpha;				// A
							if( color_type == 6 ) {
								pRead[3]= pngimage[size-1-y][x*pngdepth+3] ;
							}
						}
						other = 0 ;
#endif
					}
					if( other )  {
						fread(&pRead[2],1,1,fp);	// B
						fread(&pRead[1],1,1,fp);	// G	
						fread(&pRead[0],1,1,fp);	// R
						pRead[3] = alpha;				// A
						if( isTGA && tgah.depth == 32 ) {
							fread(&pRead[3],1,1,fp);	// A
							if( alpha < pRead[3] ) pRead[3] = alpha ;
						}
					}
				}
				else {
					if( isPNG ) {
#if DEF_USE_LIBPNG
						if( color_type == 6 ) { //トゥルーカラー＋アルファ
							pRead[3]= pngimage[size-1-y][x*pngdepth+3] ;
						}
						if( color_type == 4 ) { //グレースケール＋アルファ
							pRead[3]= pngimage[size-1-y][x*pngdepth+1] ;
						}
						if( alpha < pRead[3] ) pRead[3] = alpha ;
#endif
					}
					if( isTGA ) {
						if( tgah.depth == 32 ) { //いらないデータを読み飛ばす
							fread(wbuf,3,1,fp);	// BGR
						}
						fread(&pRead[3],1,1,fp);	// A
						if( alpha < pRead[3] ) pRead[3] = alpha ;
					}
				}
				pRead+=4;
			}
		}
		fclose(fp);
		fp = NULL ;
	}
	if( sts != 0 ) {
		if( pImage != NULL ) free(pImage) ;
		if( fp != NULL ) fclose(fp) ;
	}
#if DEF_USE_LIBPNG
	if( pngimage != NULL ) {
		unsigned int uy ;
		for (uy = 0; uy < pngheight; uy++) free(pngimage[uy]);            // 以下２行は２次元配列を解放します
		free(pngimage);
	}
#endif
#if DEF_USE_LIBJPEG
	if( jpegimage != NULL ) {
		unsigned int uy ;
		for (uy = 0; uy < cinfo.output_height; uy++) free(jpegimage[uy]);            // 以下２行は２次元配列を解放します
		free(jpegimage);
	}
#endif
	if( size < 0 ) {
		if( pImage != NULL ) free(pImage) ;
		pImage = NULL ;
	}
	*tex_size = size;

	return pImage;
}

/*=========================================================================
【関数】mqoLoadFile
【用途】メタセコイアファイル(*.mqo)からモデルデータをOpenGLに読み込む
【引数】
		filename	ファイルのパス
		scale		拡大率

【戻値】成功：1 ／ 失敗：0
=========================================================================*/
int mqoLoadFile(STR_MQO_OBJECT *mqoobj,char *filename,double scale,unsigned char alpha)
{
	FILE			*fp;
	MQO_OBJECT_V2		obj[N_OBJ];
	MQO_MATERIAL	*M = NULL;

	char	buf[SIZE_STR];
	char	path_dir[SIZE_STR];	// ディレクトリのパス
	char	path_tex[SIZE_STR];	// テクスチャファイルのパス
	char path_alp[SIZE_STR];	// アルファテクスチャファイルのパス
	int		n_mat = 0;			// マテリアル数
	int		n_obj = 0;			// オブジェクト数
	int		i;

	// MaterialとObjectの読み込み
	fp = fopen(filename,"rb");
	if (fp==NULL) return 0;

	mqoobj->alpha = alpha ;

	memset(obj,0,sizeof(obj)) ;

	i = 0;
	while ( !feof(fp) ) {
		fgets(buf,SIZE_STR,fp);

		// Material
		if (strstr(buf,"Material")) {
			sscanf(buf,"Material %d", &n_mat);
			M = (MQO_MATERIAL*) calloc( n_mat, sizeof(MQO_MATERIAL) );
			mqoReadMaterial(fp,M);
		}

		// Object
		if (strstr(buf,"Object")) {
			sscanf(buf,"Object %s", obj[i].objname);
			mqoReadObject(fp, &obj[i]);
			i++;
		}
	}
	n_obj = i;
	fclose(fp);

	// パスの取得
	mqoGetDirectory(filename, path_dir);

	// テクスチャの登録
	for (i=0; i<n_mat; i++) {
		if (M[i].useTex) {
			if (strstr(M[i].texFile,":")) {
				strcpy(path_tex, M[i].texFile);	// 絶対パスの場合
			} else {
				sprintf(path_tex,"%s%s",path_dir,M[i].texFile);	// 相対パスの場合
			}
			if( M[i].alpFile[0] != '\0' ) {
				if (strstr(M[i].texFile,":")) {
					strcpy(path_alp, M[i].alpFile);	// 絶対パスの場合
				} else {
					sprintf(path_alp,"%s%s",path_dir,M[i].alpFile);	// 相対パスの場合
				}
				M[i].texName = mqoSetTexturePool(path_tex,path_alp,alpha) ;
			}
			else {
				M[i].texName = mqoSetTexturePool(path_tex,NULL,alpha) ;
			}
		}
	}

	mqoMakeObjectsEx(mqoobj,obj,n_obj,M,n_mat,scale,alpha) ;

	// オブジェクトのデータの開放
	for (i=0; i<n_obj; i++) {
		free(obj[i].V);
		free(obj[i].F);
	}

	// マテリアルの開放
	free(M);

	return 1;
}

/*=========================================================================
【関数】mqoCreateList
【用途】MQOオブジェクトを指定数分確保する
【引数】num		MQOオブジェクトの数

【戻値】MQOオブジェクト
【仕様】MQOオブジェクト（MQO_OBJECT）頂点配列使用時は頂点情報の構造体です。
=========================================================================*/
STR_MQO_OBJECT * mqoCreateList(int num)
{
	STR_MQO_OBJECT * displist;
	if( ! l_GLMetaseqInitialized ) GLMetaseqInitialize() ;//一応、初期化されてなかったら初期化する処理追加
	displist = (STR_MQO_OBJECT *)malloc(sizeof(STR_MQO_OBJECT)*num) ;
	memset(displist,0,sizeof(STR_MQO_OBJECT)*num) ;

	return displist;
}
/*=========================================================================
【関数】mqoCreateListObject
【用途】メタセコイアファイル(*.mqo)からMQOオブジェクトを作成する
【引数】displist	MQOオブジェクト
		num			読み込み先番号（ディスプレイリスト＋numにMQOファイルを読み込む）
		filename	ファイルのパス
		scale		拡大率
		alpha		アルファ指定（全体のアルファ値を指定（０〜２５５））

【戻値】ステータス　負：異常　０：正常
【仕様】MQOオブジェクト（MQO_OBJECT）頂点配列使用時は頂点情報の構造体です。
=========================================================================*/
int mqoCreateListObject(STR_MQO_OBJECT * displist,int num,char *filename,double scale,unsigned char alpha)
{
	int ret ;
	ret = 0 ;
	if( displist == (STR_MQO_OBJECT *)NULL ) return -1 ;
	if (! mqoLoadFile(&displist[num],filename,scale,alpha)) ret = -1 ; 
	return ret;
}
/*=========================================================================
【関数】mqoCallObject
【用途】MQOオブジェクトをOpenGLの画面上に呼び出す
【引数】
		object		MQOオブジェクト
		num			リスト番号

【戻値】なし
=========================================================================*/
void mqoCallListObject(STR_MQO_OBJECT * object,int num)
{
	int o ;
	int m ;
	char *base ;
	int offset ;
	STR_MQO_INNER_OBJECT *obj ;
	STR_MATERIAL *mat ;
	double dalpha ;
	GLfloat lw[4] ;
	//メタセコは頂点の並びが表面からみて右回り
	GLint		intFrontFace ;
	GLboolean			isGL_DEPTH_TEST = GL_FALSE ;
	glPushMatrix();
		//メタセコは頂点の並びが表面からみて右回り
		glGetIntegerv(GL_FRONT_FACE,&intFrontFace) ;
		glFrontFace(GL_CW) ;
		isGL_DEPTH_TEST = glIsEnabled(GL_DEPTH_TEST) ;
		if( isGL_DEPTH_TEST == GL_FALSE ) {
			glEnable(GL_DEPTH_TEST);
		}
//		glRotatef(90.0f, 1.f, 0.f, 0.f); // ARToolKitではZ軸が上方向になるので
		dalpha = (double)object[num].alpha/(double)255 ;
		for( o = 0 ; o < object[num].objnum ; o++ ) {//内部オブジェクトループ
			obj = &object[num].obj[o] ;
			if( ! obj->isVisible ) continue ;
			for( m = 0 ; m < obj->matnum ; m++ ) {//マテリアルループ
				mat = &obj->mat[m] ;
				if( mat->datanum == 0 ) continue ;
				if( mat->isValidMaterialInfo ) {//マテリアルの情報設定
					/*
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat->dif);
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat->amb);
					*/
					memcpy(lw,mat->dif,sizeof(lw)) ;
					lw[3]*=dalpha ;
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, lw);
					memcpy(lw,mat->amb,sizeof(lw)) ;
					lw[3]*=dalpha ;
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, lw);
					glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat->spc);
					glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat->emi);
					glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat->power);
				}
				if( mat->isUseTexture) {
					glEnableClientState( GL_VERTEX_ARRAY );
					glEnableClientState( GL_NORMAL_ARRAY );
					glEnableClientState( GL_TEXTURE_COORD_ARRAY );

//					glShadeModel(GL_SMOOTH);
					glEnable(GL_TEXTURE_2D);
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

					glBindTexture(GL_TEXTURE_2D,mat->texture_id);
					glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

					if( g_isVBOSupported ) {//頂点バッファ使用
						base = (char *)NULL ;//アドレスはNULLが先頭
#if WIN32
						glBindBufferARB( GL_ARRAY_BUFFER_ARB, mat->VBO_id );//頂点バッファを結びつける
#endif
					}
					else {
						//頂点配列の時は、アドレスをそのまま入れる
						base = (char *)mat->texdata[0].point ;
					}
					//baseからのオフセットを求める（といっても、頂点は０なんだけどね）
					offset = (int)((char *)mat->texdata[0].point-(char *)mat->texdata[0].point) ;
					//頂点配列を設定　３：三角　GL_FLOAT：GLfloat型使用　サイズ：頂点データ間のオフセット　アドレス：先頭アドレス
					//　　　　　										　（xyzで一頂点、次のxyzまでのオフセット）
					glVertexPointer( 3, GL_FLOAT, sizeof(STR_MATERIAL_TEX_USE) , base+offset );
					//baseからのオフセットを求める（uvのアドレス−頂点のアドレス＝baseからｕｖまでのオフセット）
					offset = (int)((char *)mat->texdata[0].uv-(char *)mat->texdata[0].point) ;
					glTexCoordPointer( 2, GL_FLOAT, sizeof(STR_MATERIAL_TEX_USE) , base+offset );
					offset = (int)((char *)mat->texdata[0].normal-(char *)mat->texdata[0].point) ;
					glNormalPointer( GL_FLOAT, sizeof(STR_MATERIAL_TEX_USE) , base+offset );

					glColor4f(mat->mat_color[0],mat->mat_color[1],mat->mat_color[2],mat->mat_color[3]*dalpha) ;

					//描画実行
					glDrawArrays( GL_TRIANGLES, 0, mat->datanum );

					glDisable(GL_BLEND);
					glDisable(GL_TEXTURE_2D);
					glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
					if( g_isVBOSupported ) {//頂点バッファ使用
#if WIN32
						glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );//頂点バッファをデフォルトへ
#endif
					}

					glDisableClientState( GL_VERTEX_ARRAY );
					glDisableClientState( GL_NORMAL_ARRAY );
					glDisableClientState( GL_TEXTURE_COORD_ARRAY );
				}
				else {

					glEnableClientState( GL_VERTEX_ARRAY );
					glEnableClientState( GL_NORMAL_ARRAY );
//					glEnableClientState( GL_COLOR_ARRAY );

					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

					if( g_isVBOSupported ) {
						base = (char *)NULL ;
#if WIN32
						glBindBufferARB( GL_ARRAY_BUFFER_ARB, mat->VBO_id );
#endif
					}
					else {
						base = (char *)mat->matdata[0].point ;
					}
					offset = (int)((char *)mat->matdata[0].point-(char *)mat->matdata[0].point) ;
					glVertexPointer( 3, GL_FLOAT, sizeof(STR_MATERIAL_ONLY) , base+offset );
					offset = (int)((char *)mat->matdata[0].normal-(char *)mat->matdata[0].point) ;
					glNormalPointer( GL_FLOAT, sizeof(STR_MATERIAL_ONLY) , base+offset );

					glColor4f(mat->mat_color[0],mat->mat_color[1],mat->mat_color[2],mat->mat_color[3]*dalpha) ;
//					offset = (int)((char *)mat->matdata[0].color-(char *)mat->matdata[0].point) ;
//					glColorPointer(4,GL_FLOAT,sizeof(STR_MATERIAL_ONLY),base+offset) ;

					glDrawArrays( GL_TRIANGLES, 0, mat->datanum );

					glDisable(GL_BLEND);
					if( g_isVBOSupported ) {//頂点バッファ使用
#if WIN32
						glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );//頂点バッファをデフォルトへ
#endif
					}

//					glDisableClientState( GL_COLOR_ARRAY );
					glDisableClientState( GL_VERTEX_ARRAY );
					glDisableClientState( GL_NORMAL_ARRAY );

				}
			}
		}
		//メタセコは頂点の並びが表面からみて右回り（元の設定にもどす）
		glFrontFace(intFrontFace) ;

		if( isGL_DEPTH_TEST == GL_FALSE ) {
			glDisable(GL_DEPTH_TEST);
		}
	glPopMatrix();
}
//MQO_OBJECTのクリア
void mqoClearObject(STR_MQO_OBJECT * object,int from,int num) 
{
{
	int loop ;
	int o ;
	int m ;
	STR_MQO_INNER_OBJECT *obj ;
	STR_MATERIAL *mat ;
	if( object == NULL ) return ;
	for( loop = from ; loop < from +num ; loop++ ) {
		for( o = 0 ; o < (object+from)->objnum ; o++ ) {
			obj = &(object+loop)->obj[o] ;
			for( m = 0 ; m < obj->matnum ; m++ ) {
				mat = &obj->mat[m] ;
				if( mat->datanum <= 0 ) continue ;
				if( g_isVBOSupported ) {
#if WIN32
					//頂点バッファの削除
					glDeleteBuffersARB( 1, &mat->VBO_id );
#endif
				}
				//頂点配列の削除
				if( mat->isUseTexture ) {
					if( mat->texdata != NULL ) free(mat->texdata) ;
				}
				else {
					if( mat->matdata != NULL ) free(mat->matdata) ;
				}
			}
			free(obj->mat) ;
			obj->matnum = 0 ;
		}
	}
}
}
//MQO_MODELの削除
// MQO_MODEL object	:削除するMQO_MODEL
// int num				:削除する個数
void mqoDeleteObject(STR_MQO_OBJECT * object,int num)
{
	mqoClearObject(object,0,num) ;
	free(object) ;
}

//MQO_MODELの内部にデータを設定する
void mqoSet(E_MQO mode,STR_MQO_OBJECT * object,...) 
{
	va_list args ;

	va_start(args,object) ;
	switch(mode) {
	case E_MQO_VISIBLE :
		//　可変部：	int 内部オブジェクト配列の位置
		//				int ０：非表示／その他：表示
		{
			int objpos ;
			int  isVisible ;
			objpos = (int)va_arg(args,int) ;
			isVisible = (int)va_arg(args,int) ;
			object->obj[objpos].isVisible = isVisible ;
		}
		break ;
	case E_MQO_ALPHA :
		//　可変部：		unsigned char * アルファ値（０〜２５５）
		{
			unsigned char *uc ;
			uc = (unsigned char *)va_arg(args,unsigned char *) ;
			object->alpha = *uc ;
		}
		break ;
	default :
		printf("mqoSet : 未定義の処理[%d]\n",mode) ;
		break ;
	}

	va_end(args) ;
}
//MQO_OBJECTの内部データを取り出す
void mqoGet(E_MQO mode,STR_MQO_OBJECT * object,...) 
{
	va_list args ;

	va_start(args,object) ;
	switch(mode) {
	case E_MQO_VISIBLE :
		//　可変部：in		int 内部オブジェクト配列の位置
		//			out		char * 内部オブジェクト名
		//			out		int  * ０：非表示／その他：表示
		{
			int pos ;
			char *name ;
			int  *isVisible ;
			pos =  (int)va_arg(args,int) ;
			if( object->objnum <= pos ) {
				break ;
			}
			name = (char *)va_arg(args,char *) ;
			isVisible = (int *)va_arg(args,int *) ;
			strcpy(name,object->obj[pos].objname) ;
			*isVisible = object->obj[pos].isVisible ;
		}
		break ;
	case E_MQO_ALPHA :
		//　可変部：out		unsigned char * アルファ値（０〜２５５）
		{
			unsigned char *uc ;
			uc = (unsigned char *)va_arg(args,unsigned char *) ;
			*uc = object->alpha ;
		}
		break ;
	default :
		printf("mqoGet : 未定義の処理[%d]\n",mode) ;
		break ;
	}

	va_end(args) ;
}

////////////////////////////////////////////////////////////////////////////////////////////
// 関数本体


/*=========================================================================
【関数】mqoGetDirectory
【用途】ファイル名を含むパス文字列からディレクトリのパスのみを抽出する
【引数】
		*path_file	ファイル名を含むパス文字列（入力）
		*path_dir	ファイル名を除いたパス文字列（出力）

【戻値】なし
【仕様】例：
		"C:/data/file.bmp" → "C:/data/"
		"data/file.mqo"    → "data/"
=========================================================================*/

void mqoGetDirectory(const char *path_file, char *path_dir)
{
	char *pStr;
	int len;

	pStr = MAX( strrchr(path_file,'\\'), strrchr(path_file,'/') );
	len = MAX((int)(pStr-path_file)+1,0);
	strncpy(path_dir,path_file,len);
	path_dir[len] = '\0';
}






/*=========================================================================
【関数】mqoSnormal
【用途】法線ベクトルを求める
【引数】
		A		3次元座標上の点A
		B		3次元座標上の点B
		C		3次元座標上の点C
		*normal	ベクトルBAとベクトルBCの法線ベクトル（右ねじ方向）

【戻値】なし
【仕様】メタセコイアにおいて面を構成する頂点の番号は，表示面から見て
		時計回りに記述してある．つまり，頂点A,B,C があったとき，
		求めるべき法線はBAとBCの外積によって求められる
=========================================================================*/

void mqoSnormal(glPOINT3f A, glPOINT3f B, glPOINT3f C, glPOINT3f *normal)
{
	double norm;
	glPOINT3f vec0,vec1;

	// ベクトルBA
	vec0.x = A.x - B.x; 
	vec0.y = A.y - B.y;
	vec0.z = A.z - B.z;

	// ベクトルBC
	vec1.x = C.x - B.x;
	vec1.y = C.y - B.y;
	vec1.z = C.z - B.z;

	// 法線ベクトル
	normal->x = vec0.y * vec1.z - vec0.z * vec1.y;
	normal->y = vec0.z * vec1.x - vec0.x * vec1.z;
	normal->z = vec0.x * vec1.y - vec0.y * vec1.x;

	// 正規化
	norm = normal->x * normal->x + normal->y * normal->y + normal->z * normal->z;
	norm = sqrt ( norm );

	normal->x /= norm;
	normal->y /= norm;
	normal->z /= norm;
}



/*=========================================================================
【関数】mqoReadMaterial
【用途】マテリアル情報の読み込み
【引数】
		fp		ファイルポインタ
		M		マテリアル配列

【戻値】なし
【仕様】mqoCreateModel(), mqoCreateSequence()のサブ関数．
=========================================================================*/

void mqoReadMaterial(FILE *fp, MQO_MATERIAL M[])
{
	GLfloat		dif, amb, emi, spc;
	glCOLOR4f	c;
	char		buf[SIZE_STR];
	char		*pStrEnd, *pStr;
	int			len;
	int			i = 0;

	while (1) {
		fgets(buf,SIZE_STR,fp);	// 行読み込み
		if (strstr(buf,"}")) break;

		pStr = strstr(buf,"col(");	// 材質名読み飛ばし
		sscanf( pStr,
				"col(%f %f %f %f) dif(%f) amb(%f) emi(%f) spc(%f) power(%f)",
				&c.r, &c.g, &c.b, &c.a, &dif, &amb, &emi, &spc, &M[i].power );

		// 頂点カラー
		M[i].col = c;

		// 拡散光
		M[i].dif[0] = dif * c.r;
		M[i].dif[1] = dif * c.g;
		M[i].dif[2] = dif * c.b;
		M[i].dif[3] = c.a;

		// 周囲光
		M[i].amb[0] = amb * c.r;
		M[i].amb[1] = amb * c.g;
		M[i].amb[2] = amb * c.b;
		M[i].amb[3] = amb * c.a;

		// 自己照明
		M[i].emi[0] = emi * c.r;
		M[i].emi[1] = emi * c.g;
		M[i].emi[2] = emi * c.b;
		M[i].emi[3] = emi * c.a;

		// 反射光
		M[i].spc[0] = spc * c.r;
		M[i].spc[1] = spc * c.g;
		M[i].spc[2] = spc * c.b;
		M[i].spc[3] = spc * c.a;

		
		// tex：模様マッピング名
		if ( (pStr = strstr(buf,"tex(")) != NULL ) {
			M[i].useTex = TRUE;

			pStrEnd = strstr(pStr,"\")");
			len = pStrEnd - (pStr+5);
			strncpy(M[i].texFile,pStr+5,len);
			M[i].texFile[len] = '\0';
			if ( (pStr = strstr(buf,"aplane(")) != NULL ) {
				pStrEnd = strstr(pStr,"\")");
				len = pStrEnd - (pStr+8);
				strncpy(M[i].alpFile,pStr+8,len);
				M[i].alpFile[len] = '\0';
			}
			else {
				M[i].alpFile[0] = '\0' ;
			}

		} else {
			M[i].useTex = FALSE;
			M[i].texFile[0] = '\0';
			M[i].alpFile[0] = '\0' ;
		}

		i++;
	}

}



/*=========================================================================
【関数】mqoReadVertex
【用途】頂点情報の読み込み
【引数】
		fp		ファイルポインタ
		V		頂点配列
		
【戻値】なし
【仕様】mqoReadObject()のサブ関数．
=========================================================================*/

void mqoReadVertex(FILE *fp, glPOINT3f V[])
{
	char buf[SIZE_STR];
	int  i=0;

	while (1) {
		fgets(buf,SIZE_STR,fp);
		if (strstr(buf,"}")) break;
		sscanf(buf,"%f %f %f",&V[i].x,&V[i].y,&V[i].z);
		i++;
	}
}
/*=========================================================================
【関数】mqoReadBVertex
【用途】メタセコイアファイルからバイナリ形式の頂点情報を読み込む

【引数】*fp			現在オープンしているメタセコイアファイルのファイルポインタ
		V[]			頂点を格納する配列
		max_vertex	頂点数の上限

【戻値】頂点数
【仕様】mqoLoadFileの子関数なのでこれ単体で使うことはない
=========================================================================*/

int mqoReadBVertex(FILE *fp,glPOINT3f V[])
{
	int n_vertex,i;
	float *wf ;
	int size ;
	char cw[256] ;
	char *pStr ;

	fgets(cw,sizeof(cw),fp) ;
	if ( (pStr = strstr(cw,"Vector")) != NULL ) {
		sscanf(pStr,"Vector %d [%d]",&n_vertex,&size);				// 頂点数、データサイズを読み込む
	}
	else {
		return -1 ;
	}
	//MQOファイルのバイナリ頂点データはintel形式（リトルエディアン）
	wf = (float *)malloc(size) ;
	fread(wf,size,1,fp) ;
	for( i = 0 ; i < n_vertex ; i++ ) {
		V[i].x = wf[i*3+0] ;
		V[i].y = wf[i*3+1] ;
		V[i].z = wf[i*3+2] ;
#if DEF_IS_LITTLE_ENDIAN
#else
		endianConverter((void *)&V[i].x,sizeof(V[i].x)) ;
		endianConverter(&V[i].y,sizeof(V[i].y)) ;
		endianConverter(&V[i].z,sizeof(V[i].z)) ;
#endif
	}
	free(wf) ;

	// "}"まで読み飛ばし
	{
		char buf[SIZE_STR] ;
		while (1) {
			fgets(buf,SIZE_STR,fp);
			if (strstr(buf,"}")) break;
		}
	}

	return n_vertex;
}



/*=========================================================================
【関数】mqoReadFace
【用途】面情報の読み込み
【引数】
		fp		ファイルポインタ
		F		面配列
		
【戻値】なし
【仕様】mqoReadObject()のサブ関数．
=========================================================================*/

void mqoReadFace(FILE *fp, MQO_FACE F[])
{
	char buf[SIZE_STR];
	char *pStr;
	int  i=0;

	while (1) {
		fgets(buf,SIZE_STR,fp);
		if (strstr(buf,"}")) break;

		// 面を構成する頂点数
		sscanf(buf,"%d",&F[i].n);

		// 頂点(V)の読み込み
		if ( (pStr = strstr(buf,"V(")) != NULL ) {
			switch (F[i].n) {
				case 3:
					sscanf(pStr,"V(%d %d %d)",&F[i].v[0],&F[i].v[1],&F[i].v[2]);
					break;
				case 4:
					sscanf(pStr,"V(%d %d %d %d)",&F[i].v[0],&F[i].v[1],&F[i].v[2],&F[i].v[3]);
					break;
				default:
					break;
			}		
		}

		// マテリアル(M)の読み込み
		F[i].m = 0;
		if ( (pStr = strstr(buf,"M(")) != NULL ) {
			sscanf(pStr,"M(%d)",&F[i].m);
		}

		// UVマップ(UV)の読み込み
		if ( (pStr = strstr(buf,"UV(")) != NULL ) {
			switch (F[i].n) {
				case 3:	// 頂点数3
					sscanf(pStr,"UV(%f %f %f %f %f %f)",
									&F[i].uv[0].x, &F[i].uv[0].y,
									&F[i].uv[1].x, &F[i].uv[1].y,
									&F[i].uv[2].x, &F[i].uv[2].y
									);
					break;

				case 4:	// 頂点数4
					sscanf(pStr,"UV(%f %f %f %f %f %f %f %f)",
									&F[i].uv[0].x, &F[i].uv[0].y,
									&F[i].uv[1].x, &F[i].uv[1].y,
									&F[i].uv[2].x, &F[i].uv[2].y,
									&F[i].uv[3].x, &F[i].uv[3].y
									);
					break;
				default:
					break;
			}		
		}

		i++;
	}

}



/*=========================================================================
【関数】mqoReadObject
【用途】オブジェクト情報の読み込み
【引数】
		fp		ファイルポインタ
		obj		オブジェクト情報

【戻値】なし
【仕様】mqoCreateModel(), mqoCreateSequence()のサブ関数．
		この関数で１個のオブジェクト情報が読み込まれる．
=========================================================================*/

void mqoReadObject(FILE *fp, MQO_OBJECT_V2 *obj)
{
	char buf[SIZE_STR];

	while (1) {
		fgets(buf,SIZE_STR,fp);
		if (strstr(buf,"}")) break;

		// visible
		if (strstr(buf,"visible ")) {
			sscanf(buf," visible %d", &obj->visible);
		}

		// facet
		if (strstr(buf,"facet ")) {
			sscanf(buf," facet %f", &obj->facet);
		}

		// vertex
		if (strstr(buf,"vertex ")) {
			sscanf(buf," vertex %d", &obj->n_vertex);
			obj->V = (glPOINT3f*) calloc( obj->n_vertex, sizeof(glPOINT3f) );
			mqoReadVertex(fp, obj->V);
		}
		// BVertex
		if (strstr(buf,"BVertex"))	{
			sscanf(buf," BVertex %d", &obj->n_vertex);
			obj->V = (glPOINT3f*) calloc( obj->n_vertex, sizeof(glPOINT3f) );
			mqoReadBVertex(fp,obj->V);
		}

		// face
		if (strstr(buf,"face ")) {
			sscanf(buf," face %d", &obj->n_face);
			obj->F = (MQO_FACE*) calloc( obj->n_face, sizeof(MQO_FACE) );
			mqoReadFace(fp, obj->F);
		}

	}

}



/*=========================================================================
【関数】mqoMakePolygon
【用途】ポリゴンの作成
【引数】
		*F		面情報
		V[]		頂点配列
		M[]		マテリアル配列
【戻値】なし
【仕様】mqoMakeObjects()のサブ関数．
		この関数で１枚のポリゴンが作られる．
=========================================================================*/

//頂点配列の作成
//頂点配列はすべて三角にするので、四角は三角ｘ２に分割
//  0  3      0    0  3
//   □   →　△　　▽
//  1  2     1  2   2
void mqoMakeArray(STR_MATERIAL *mat,int matpos,MQO_FACE F[],int fnum,glPOINT3f V[],glPOINT3f N[],double facet,glCOLOR4f *mcol,double scale,unsigned char alpha)
{
	int f ;
	int i ;
	int dpos ;
	double s ;
	glPOINT3f normal;	// 法線ベクトル
	
	dpos = 0 ;
	mat->mat_color[0] = mcol->r ;
	mat->mat_color[1] = mcol->g ;
	mat->mat_color[2] = mcol->b ;
	mat->mat_color[3] = mcol->a ;
	if( mat->isUseTexture ) {
		for( f = 0 ; f < fnum ; f++ ){
			if( F[f].m != matpos ) continue ;
			if( F[f].n == 3 ) {
				mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);	// 法線ベクトルを計算
				for( i = 0 ; i < 3 ; i++ ) {
					mat->texdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->texdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->texdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->texdata[dpos].uv[0] = F[f].uv[i].x ;
					mat->texdata[dpos].uv[1] = F[f].uv[i].y ;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->texdata[dpos].normal[0] = normal.x ;
						mat->texdata[dpos].normal[1] = normal.y ;
						mat->texdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->texdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->texdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->texdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
			}
			//４頂点（四角）は３頂点（三角）ｘ２に分割
			if( F[f].n == 4 ) {
				mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);	// 法線ベクトルを計算
				for( i = 0 ; i < 4 ; i++ ) {
					if( i == 3 ) continue ;
					mat->texdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->texdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->texdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->texdata[dpos].uv[0] = F[f].uv[i].x ;
					mat->texdata[dpos].uv[1] = F[f].uv[i].y ;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->texdata[dpos].normal[0] = normal.x ;
						mat->texdata[dpos].normal[1] = normal.y ;
						mat->texdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->texdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->texdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->texdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
				mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&normal);	// 法線ベクトルを計算
				for( i = 0 ; i < 4 ; i++ ) {
					if( i == 1 ) continue ;
					mat->texdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->texdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->texdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->texdata[dpos].uv[0] = F[f].uv[i].x ;
					mat->texdata[dpos].uv[1] = F[f].uv[i].y ;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->texdata[dpos].normal[0] = normal.x ;
						mat->texdata[dpos].normal[1] = normal.y ;
						mat->texdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->texdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->texdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->texdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
			}
		}
	}
	else {
		if( alpha != 255 ) {
			mat->mat_color[3] = (double)alpha/(double)255 ;
		}
		for( f = 0 ; f < fnum ; f++ ){
			if( F[f].m != matpos ) continue ;
			if( F[f].n == 3 ) {
				mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);					// 法線ベクトルを計算
				for( i = 0 ; i < 3 ; i++ ) {
					mat->matdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->matdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->matdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->matdata[dpos].normal[0] = normal.x;
					mat->matdata[dpos].normal[1] = normal.y;
					mat->matdata[dpos].normal[2] = normal.z;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->matdata[dpos].normal[0] = normal.x ;
						mat->matdata[dpos].normal[1] = normal.y ;
						mat->matdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->matdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->matdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->matdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
			}
			//４頂点（四角）は３頂点（三角）ｘ２に分割
			if( F[f].n == 4 ) {
				mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);					// 法線ベクトルを計算
				for( i = 0 ; i < 4 ; i++ ) {
					if( i == 3 ) continue ;
					mat->matdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->matdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->matdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->matdata[dpos].normal[0] = normal.x;
					mat->matdata[dpos].normal[1] = normal.y;
					mat->matdata[dpos].normal[2] = normal.z;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->matdata[dpos].normal[0] = normal.x ;
						mat->matdata[dpos].normal[1] = normal.y ;
						mat->matdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->matdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->matdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->matdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
				mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&normal);					// 法線ベクトルを計算
				for( i = 0 ; i < 4 ; i++ ) {
					if( i == 1 ) continue ;
					mat->matdata[dpos].point[0] = V[F[f].v[i]].x*scale ;
					mat->matdata[dpos].point[1] = V[F[f].v[i]].y*scale ;
					mat->matdata[dpos].point[2] = V[F[f].v[i]].z*scale ;
					mat->matdata[dpos].normal[0] = normal.x;
					mat->matdata[dpos].normal[1] = normal.y;
					mat->matdata[dpos].normal[2] = normal.z;
					s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z) ;
					if( facet < s ) {
						mat->matdata[dpos].normal[0] = normal.x ;
						mat->matdata[dpos].normal[1] = normal.y ;
						mat->matdata[dpos].normal[2] = normal.z ;
					}
					else {
						mat->matdata[dpos].normal[0] = N[F[f].v[i]].x ;
						mat->matdata[dpos].normal[1] = N[F[f].v[i]].y ;
						mat->matdata[dpos].normal[2] = N[F[f].v[i]].z ;
					}
					dpos++ ;
				}
			}
		}
	}
}
//頂点法線の計算（４頂点の面は▽に分割して計算）
//リターン値は呼び出し元でfreeすること！
glPOINT3f *mqoVertexNormal(MQO_OBJECT_V2 *obj) {
	int f ;
	int v ;
	int i ;
	double len ;
	glPOINT3f fnormal;	// 面法線ベクトル
	MQO_FACE *F ;
	glPOINT3f *V ;
	glPOINT3f *ret ;
	F = obj->F ;
	V = obj->V ;
	ret = (glPOINT3f *)calloc(obj->n_vertex,sizeof(glPOINT3f)) ;
	//面の法線を頂点に足し込み
	for( f = 0 ; f < obj->n_face ; f++ ) {
		if( obj->F[f].n == 3 ) {
			mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&fnormal);
			for( i = 0 ; i < 3 ; i++ ) {
				ret[F[f].v[i]].x += fnormal.x ;
				ret[F[f].v[i]].y += fnormal.y ;
				ret[F[f].v[i]].z += fnormal.z ;
			}
		}
		if( obj->F[f].n == 4 ) {
			mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&fnormal);
			for( i = 0 ; i < 4 ; i++ ) {
				if( i == 3 ) continue ;
				ret[F[f].v[i]].x += fnormal.x ;
				ret[F[f].v[i]].y += fnormal.y ;
				ret[F[f].v[i]].z += fnormal.z ;
			}
			mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&fnormal);
			for( i = 0 ; i < 4 ; i++ ) {
				if( i == 1 ) continue ;
				ret[F[f].v[i]].x += fnormal.x ;
				ret[F[f].v[i]].y += fnormal.y ;
				ret[F[f].v[i]].z += fnormal.z ;
			}
		}
	}
	//正規化
	for( v = 0 ; v < obj->n_vertex ; v++ ) {
		if( ret[v].x == 0 && ret[v].y == 0 && ret[v].z == 0 ) {
			//面に使われてない点
			continue ;
		}
		len = sqrt(ret[v].x*ret[v].x + ret[v].y*ret[v].y + ret[v].z*ret[v].z) ;
		if( len != 0 ) {
			ret[v].x = ret[v].x/len ;
			ret[v].y = ret[v].y/len ;
			ret[v].z = ret[v].z/len ;
		}
	}

	return ret ;
}
//マテリアル毎に頂点配列を作成する
void mqoMakePolygonEx(char *objname,int isVisible,STR_MQO_OBJECT * mqoobj,
					  MQO_FACE F[],int fnum,glPOINT3f V[],glPOINT3f N[],MQO_MATERIAL M[],
					  int n_mat,double facet,double scale,unsigned char alpha)
{
	int f ;
	int m ;
	int *mat_vnum ;
	STR_MQO_INNER_OBJECT *setObj ;
	STR_MATERIAL *material ;
	glCOLOR4f defcol ;
	glCOLOR4f *pcol ;

	setObj = &mqoobj->obj[mqoobj->objnum] ;
	strcpy(setObj->objname,objname) ;
	setObj->isVisible = isVisible ;

	//faceの中でのマテリアル毎の頂点の数
	// M=NULLのとき、F[].m = 0 が入ってくる
	if( M == NULL ) n_mat = 1 ;

	mat_vnum = (int *)malloc(sizeof(int)*n_mat) ;
	memset(mat_vnum,0,sizeof(int)*n_mat) ;
	for( f = 0 ; f < fnum ; f++ ){
		if( F[f].n == 3 ) {
			mat_vnum[F[f].m] += 3 ;
		}
		if( F[f].n == 4 ) {
			//４頂点（四角）は３頂点（三角）ｘ２に分割
			//  0  3      0    0  3
			//   □   →　△　　▽
			//  1  2     1  2   2
			// ４頂点の平面データは
			// ３頂点の平面データｘ２個
			mat_vnum[F[f].m] += 3*2 ;
		}
		if( setObj->matnum < F[f].m+1 ) setObj->matnum = F[f].m+1 ;
	}
	//マテリアル別に頂点配列を作成する
	setObj->mat = (STR_MATERIAL *)malloc(sizeof(STR_MATERIAL)*setObj->matnum) ;
	memset(setObj->mat,0,sizeof(STR_MATERIAL)*setObj->matnum) ;
	for( m = 0 ; m < setObj->matnum ; m++ ) {
		material = &setObj->mat[m] ;
		material->datanum = mat_vnum[m] ;
		material->isValidMaterialInfo = (M != NULL) ;
		if( mat_vnum[m] <= 0 ) continue ;
		if( material->isValidMaterialInfo ) {
			memcpy(material->dif,M[m].dif,sizeof(material->dif)) ;
			memcpy(material->amb,M[m].amb,sizeof(material->amb)) ;
			memcpy(material->spc,M[m].spc,sizeof(material->spc)) ;
			memcpy(material->emi,M[m].emi,sizeof(material->emi)) ;
			material->power = M[m].power ;
			material->isUseTexture = M[m].useTex ;
			pcol = &M[m].col ;
		}
		else {
			defcol.r = 1.0 ;
			defcol.g = 1.0 ;
			defcol.b = 1.0 ;
			defcol.a = 1.0 ;
			material->isUseTexture = 0 ;
			pcol = &defcol ;
		}
		if( material->isUseTexture ) {
			material->texdata = (STR_MATERIAL_TEX_USE *)calloc(material->datanum,sizeof(STR_MATERIAL_TEX_USE)) ;
			material->texture_id = M[m].texName ;
		}
		else {
			material->matdata = (STR_MATERIAL_ONLY *)calloc(material->datanum,sizeof(STR_MATERIAL_ONLY)) ;
		}
		mqoMakeArray(material,m,F,fnum,V,N,facet,pcol,scale,alpha) ;
		if(g_isVBOSupported) {
			if( material->isUseTexture ) {
#if WIN32
				glGenBuffersARB( 1, &material->VBO_id );
				glBindBufferARB( GL_ARRAY_BUFFER_ARB, material->VBO_id  );
				glBufferDataARB( GL_ARRAY_BUFFER_ARB, material->datanum*sizeof(STR_MATERIAL_TEX_USE), material->texdata, GL_STATIC_DRAW_ARB );
#endif
			}
			else {
#if WIN32
				glGenBuffersARB( 1, &material->VBO_id );
				glBindBufferARB( GL_ARRAY_BUFFER_ARB, material->VBO_id  );
				glBufferDataARB( GL_ARRAY_BUFFER_ARB, material->datanum*sizeof(STR_MATERIAL_ONLY), material->matdata, GL_STATIC_DRAW_ARB );
#endif
			}
		}
	}
	mqoobj->objnum++ ;
	if( MAX_OBJECT <= mqoobj->objnum ) {
		printf("MQOファイル読み込み：　最大オブジェクト数を超えました[%d]\n",mqoobj->objnum) ;
		mqoobj->objnum = MAX_OBJECT-1 ;
	}

	free(mat_vnum) ;

}


/*=========================================================================
【関数】mqoMakeObjects
【用途】オブジェクトのデータからポリゴンモデルを作成する
【引数】
		obj		オブジェクト配列
		n_obj	オブジェクトの個数
		M		マテリアル配列

【戻値】なし
【仕様】mqoCreateModel(), mqoCreateSequence()のサブ関数．
		オブジェクト情報，マテリアル情報を元にポリゴンモデルを作成する．
=========================================================================*/
void mqoMakeObjectsEx(STR_MQO_OBJECT *mqoobj,MQO_OBJECT_V2 obj[], int n_obj, MQO_MATERIAL M[],int n_mat,
					  double scale,unsigned int alpha)
{
	int i;
	glPOINT3f *N ;
	for (i=0; i<n_obj; i++) {
		N = mqoVertexNormal(&obj[i]) ;
			mqoMakePolygonEx(
				obj[i].objname,
				obj[i].visible,mqoobj,
				obj[i].F,
				obj[i].n_face, 
				obj[i].V, 
				N,
				M,n_mat,
				obj[i].facet,
				scale,
				alpha) ;
		free(N) ;
	}
}


/*=========================================================================
【関数】mqoCreateModel
【用途】MQOファイルからMQOモデルを作成する
【引数】
		filename	MQOファイル
		scale		拡大率（1.0でそのまま）

【戻値】MQO_MODEL（MQOモデル）
=========================================================================*/

MQO_MODEL mqoCreateModel(char *filename, double scale)
{
	MQO_MODEL ret ;
	ret = mqoCreateList(1) ;
	if( mqoCreateListObject(ret,1-1,filename,scale,(unsigned char)255) < 0 ) {
		mqoDeleteObject(ret,1) ;
		ret = NULL ;
	}
	return ret ;
}

/*=========================================================================
【関数】mqoCreateSequence
【用途】連番のMQOファイルからMQOシーケンスを作成する
【引数】
		basename	ファイル名から連番数字と".mqo"を除いたベースファイル名
		n_file		ファイル数
		scale		拡大率（1.0でそのまま）
		fade_inout	0:そのまま　正：フェードイン　負：フェードアウト
					絶対値は効果にかけるフレーム数

【戻値】MQO_SEQUENCE（MQOシーケンス）
【仕様】data0.mqo, data1.mqo, ... という感じの連番のMQOファイルを読み込んで
		MQOシーケンスを作成する．マテリアル情報は1番目のファイルからのみ
		読み込む．
=========================================================================*/
MQO_SEQUENCE mqoCreateSequenceEx(const char *basename, const char *format,int n_file,
								 double scale,int fade_inout,unsigned char alpha)
{
	MQO_SEQUENCE retSeq ;
	int iret ;
	int seq ;
	char filename[SIZE_STR] ;
	short setAlpha ;
	short calAlpha ;
	int frames ;

	retSeq.n_frame = 0 ;
	if( basename == NULL ) {
		return retSeq ;
	}
	calAlpha = alpha ;
	frames = abs(fade_inout) ;
	frames = MAX(frames,n_file) ;
	setAlpha = (fade_inout<=0)?alpha:0 ;



	retSeq.model = mqoCreateList(n_file) ;
	for( seq = 0 ; seq < frames ; seq++ ) {
		if( seq < n_file ) {
/*
			if( format == NULL ) {
				sprintf(filename,"%s",basename) ;
			}
			else {
				sprintf(filename,format,basename,seq) ;
			}
*/
			sprintf(filename,basename,seq);
		}
		if( (fade_inout !=  0) && ((frames-1) == seq) ) {
			setAlpha = (fade_inout<0)?0:calAlpha ;
		}
		iret = mqoCreateListObject(retSeq.model,seq,filename,scale,(unsigned char)setAlpha) ;
		if( iret == - 1 ) {
			seq-- ;
			mqoClearObject(retSeq.model,seq,n_file-seq) ;
			break ;
		}
		if( fade_inout !=  0 ) {
			if( fade_inout<0 ) {
				if( (n_file-seq) <= (-1*fade_inout) ) {
					setAlpha -= (calAlpha/(-1*fade_inout)) ;
					if( setAlpha < 0 ) setAlpha = 0 ;
				}
			}
			else {
				setAlpha += (calAlpha/fade_inout) ;
				if( calAlpha < setAlpha ) setAlpha = calAlpha ;
			}
		}
	}
	retSeq.n_frame = seq ;
	return retSeq ;
}
MQO_SEQUENCE mqoCreateSequence(char *basename, int n_file, double scale)
{
	return mqoCreateSequenceEx(basename,DEF_DEFAULT_FORMAT,n_file,scale,0,(unsigned char)255) ;
}



/*=========================================================================
【関数】mqoCallModel
【用途】MQOモデルをOpenGLの画面上に呼び出す
【引数】
		model		MQOモデル

【戻値】なし
=========================================================================*/

void mqoCallModel(MQO_MODEL model)
{
	mqoCallListObject(model,0) ;
}



/*=========================================================================
【関数】mqoCallSequence
【用途】MQOシーケンスをOpenGLの画面に呼び出す
【引数】
		seq		MQOシーケンス
		i		フレーム番号

【戻値】なし
【仕様】MQOシーケンスの中から指定したフレーム番号のモデルを呼び出す．
=========================================================================*/

void mqoCallSequence(MQO_SEQUENCE seq, int i)
{
	if ( i>=0 && i<seq.n_frame ) {
		mqoCallListObject(seq.model,i) ;
	}
}



/*=========================================================================
【関数】mqoDeleteModel
【用途】MQOモデルを削除する
【引数】
		model	MQOモデル

【戻値】なし
【仕様】扱いとしてはディスプレイリストの削除と同義．
=========================================================================*/
void mqoDeleteModel(MQO_MODEL model)
{
	mqoDeleteObject(model,1) ;
}



/*=========================================================================
【関数】mqoDeleteSequence
【用途】MQOシーケンスを削除する
【引数】
		seq		MQOシーケンス

【戻値】なし
【仕様】扱いとしてはディスプレイリストの削除と同義．
=========================================================================*/

void mqoDeleteSequence(MQO_SEQUENCE seq)
{
	mqoDeleteObject(seq.model, seq.n_frame) ;
}
