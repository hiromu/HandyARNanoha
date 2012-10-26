#include "OpenGLExtension.h"
#include <stdio.h>


//OpenGLの拡張がサポートされているかどうか調べる
int IsExtensionSupported( char* szTargetExtension )
{
	const unsigned char *pszExtensions = NULL;
	const unsigned char *pszStart;
	unsigned char *pszWhere, *pszTerminator;

	// Extension の名前が正しいか調べる(NULLや空白が入っちゃ駄目)
	pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
	if( pszWhere || *szTargetExtension == '\0' )
		return 0;

	// Extension の文字列を所得する
	pszExtensions = glGetString( GL_EXTENSIONS );

	// 文字列の中に必要な extension があるか調べる
	pszStart = pszExtensions;
	for(;;)
	{
		pszWhere = (unsigned char *) strstr( (const char *) pszStart, szTargetExtension );
		if( !pszWhere )
			break;
		pszTerminator = pszWhere + strlen( szTargetExtension );
		if( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
		if( *pszTerminator == ' ' || *pszTerminator == '¥0' )
			return 1;
		pszStart = pszTerminator;
	}
	return 0;
}
