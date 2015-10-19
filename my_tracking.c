#include <stdio.h>
#include <stdlib.h>

#include "alcon2010.h"


#define SEARCH_RANGE 100			/* 移動物体の探索範囲 */

bounding_box obox;				/* 前フレームにおける移動物体を囲む矩形 */
unsigned char *preImg = NULL; /* 前フレームの探索範囲の画像 */

/* レベル1用 */
void my_tracking_level1( int frameID, unsigned char *image, int width, int height, const bounding_box *box0 )
{
	int  m, n, i0, j0;
	double err, merr, e;
	int minX, minY;
	int serachWidth, serachHeight;
	int preX1, preX2;
	int preY1, preY2;
	int x1, x2;
	int y1, y2;

	/*
		画像をBMP形式で保存する場合は次の様に記述する
		save_bmp( "hoge.bmp", image, width, height );
	*/

	if( frameID == 0 )
	{
		/* 最初のフレームは追跡開始位置を記憶 */
		obox = *box0;

		/* 移動物体を囲む矩形を設定 */
		set_result( frameID, obox );

		minX = obox.x - SEARCH_RANGE;
		minY = obox.y - SEARCH_RANGE;
		serachWidth = obox.w + SEARCH_RANGE * 2;
		serachHeight = obox.h + SEARCH_RANGE * 2;
		preImg = (unsigned char *)malloc( serachWidth * serachHeight * 3 * sizeof(unsigned char) );
		// memset(preImg, 0, serachWidth * serachHeight * 3 * sizeof(unsigned char));

		/* 探索範囲の画像確保 */
		for ( n = 0; n < serachHeight; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;

			for ( m = 0; m < serachWidth; m++ ) {
				if ( m + minX >= width || m + minX < 0 ) continue;

				preImg[( m + n*serachWidth )*3 + 0] = image[ ( m + minX + ( n + minY) * width ) * 3 + 0 ];
				preImg[( m + n*serachWidth )*3 + 1] = image[ ( m + minX + ( n + minY) * width ) * 3 + 1 ];
				preImg[( m + n*serachWidth )*3 + 2] = image[ ( m + minX + ( n + minY) * width ) * 3 + 2 ];
			}
		}

	}
	else
	{
		/* 前回の物体位置を基準にして現在の物体位置を探索 */
		i0 = obox.x;
		j0 = obox.y;
		merr = 1000;

		minX = obox.x - SEARCH_RANGE;
		minY = obox.y - SEARCH_RANGE;
		serachWidth = obox.w + SEARCH_RANGE * 2;
		serachHeight = obox.h + SEARCH_RANGE * 2;

		x1 = preX1 = obox.x - minX;
		x2 = preX2 = preX1 + obox.w;
		y1 = preY1 = obox.y - minY;
		y2 = preY2 = preY1 + obox.h;

		/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索 */
		for ( n = 0; n < serachHeight; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;

			for ( m = 0; m < serachWidth; m++ ) {
				if ( m + minX >= width || m + minX < 0 ) continue;
				err = 0;

				e = preImg[ ( m + n * serachWidth ) * 3 + 0 ] - image[ ( m + minX + ( n + minY) * width ) * 3 + 0 ]; err += e * e;
				e = preImg[ ( m + n * serachWidth ) * 3 + 1 ] - image[ ( m + minX + ( n + minY) * width ) * 3 + 1 ]; err += e * e;
				e = preImg[ ( m + n * serachWidth ) * 3 + 2 ] - image[ ( m + minX + ( n + minY) * width ) * 3 + 2 ]; err += e * e;
				if ( err > merr ) {
					if ( x1 > m ) x1 = m;
					if ( x2 < m ) x2 = m;
					if ( y1 > n ) y1 = n;
					if ( y2 < n ) y2 = n;
				}
			}
		}

		if ( preX1 > x1 ) {
			i0 = x1 + minX;
		}
		if ( preX2 < x2 ) {
			i0 = x2 + minX - obox.w;
		}

		if ( preY1 > y1 ) {
			j0 = y1 + minY;
		}
		if ( preY2 < y2 ) {
			j0 = y2 + minY - obox.h;
		}

		obox.x = i0;
		obox.y = j0;
		// printf("%d %d\n",obox.x, obox.y );

		/* 移動物体を囲む矩形を保存 */
		set_result( frameID, obox );

		/* 現在フレームの探索範囲を確保 */
		minX = obox.x - SEARCH_RANGE;
		minY = obox.y - SEARCH_RANGE;
		for ( n = 0; n < serachHeight; n++ ) {
			if ( n + minY >= height ) continue;

			for ( m = 0; m < serachWidth; m++ ) {
				if ( m + minX >= width ) continue;

				preImg[( m + n*serachWidth )*3 + 0] = image[ ( m + minX + ( n + minY) * width ) * 3 + 0 ];
				preImg[( m + n*serachWidth )*3 + 1] = image[ ( m + minX + ( n + minY) * width ) * 3 + 1 ];
				preImg[( m + n*serachWidth )*3 + 2] = image[ ( m + minX + ( n + minY) * width ) * 3 + 2 ];
			}
		}
	}
}

/* レベル2用 */
void my_tracking_level2( int frameID, unsigned char *image, int width, int height )
{

}

/* レベル3用 */
void my_tracking_level3( int frameID, unsigned char *image, int width, int height )
{
}
