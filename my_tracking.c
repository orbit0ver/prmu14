#include <stdio.h>
#include <stdlib.h>

#include "alcon2010.h"

// 97.4
// 97.2
// 96.1
// 91.6
// 94.2

#define SEARCH_RANGE 50			/* 移動物体の探索範囲 */
#define MISSRANGE 10 /* 探索ミスカウントの範囲 */

#define PIX(img, x, y, w, i) ( img[( (x) + (y) * (w) ) * 3 + (i)] )

bounding_box obox;				/* 前フレームにおける移動物体を囲む矩形 */
unsigned char *preImg = NULL; /* 前フレームの探索範囲の画像 */


void getImage(const unsigned char *srcImg, const bounding_box box, int width, int height)
{
	int minX, minY;
	int destWidth, destHeight;
	int x, y;

	minX = box.x - SEARCH_RANGE;
	minY = box.y - SEARCH_RANGE;
	destWidth = box.w + SEARCH_RANGE * 2;
	destHeight = box.h + SEARCH_RANGE * 2;

	if ( preImg == NULL ) {
		preImg = (unsigned char *)malloc( destWidth * destHeight * 3 * sizeof(unsigned char) );
	}

	/* 探索範囲の画像確保 */
	for ( y = 0; y < destHeight; y++ ) {
		if ( y + minY >= height || y + minY < 0 ) continue;

		for ( x = 0; x < destWidth; x++ ) {
			if ( x + minX >= width || x + minX < 0 ) continue;

			PIX(preImg, x, y, destWidth, 0) = PIX(srcImg, x + minX, y + minY, width, 0);
			PIX(preImg, x, y, destWidth, 1) = PIX(srcImg, x + minX, y + minY, width, 1);
			PIX(preImg, x, y, destWidth, 2) = PIX(srcImg, x + minX, y + minY, width, 2);
		}
	}
}

void setXY(const unsigned char *image, bounding_box *obox, int width, int height)
{
	double e, err, merr;
	int minX, minY;
	int serachWidth, serachHeight;
	int i0, j0;
	int preX1, preX2, preY1, preY2;
	int x1, x2, y1, y2;
	int m, n;
	int missCount;
	bounding_box box;
	box = *obox;

	i0 = box.x;
	j0 = box.y;
	merr = 1000;

	minX = box.x - SEARCH_RANGE;
	minY = box.y - SEARCH_RANGE;
	serachWidth = box.w + SEARCH_RANGE * 2;
	serachHeight = box.h + SEARCH_RANGE * 2;

	x1 = preX1 = SEARCH_RANGE;
	x2 = preX2 = preX1 + box.w;
	y1 = preY1 = SEARCH_RANGE;
	y2 = preY2 = preY1 + box.h;

	/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索 */
	missCount = 0;
	for ( n = y1; n >= 0; n-- ) {
		if ( n + minY >= height || n + minY < 0 ) continue;

		for ( m = 0; m < serachWidth; m++ ) {
			if ( m + minX >= width || m + minX < 0 ) continue;
			err = 0;

			e = PIX(preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				y1 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE) break;
	}

	missCount = 0;
	for ( n = y2; n < serachHeight; n++ ) {
		if ( n + minY >= height || n + minY < 0 ) continue;

		for ( m = 0; m < serachWidth; m++ ) {
			if ( m + minX >= width || m + minX < 0 ) continue;
			err = 0;

			e = PIX(preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				y2 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	missCount = 0;
	for ( m = x1; m >= 0; m-- ) {
		if ( m + minX >= width || m + minX < 0 ) continue;

		for ( n = y1; n <= y2; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;
			err = 0;

			e = PIX(preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				x1 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	for ( m = x2; m < serachWidth; m++ ) {
		if ( m + minX >= width || m + minX < 0 ) continue;

		for ( n = y1; n <= y2; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;
			err = 0;

			e = PIX(preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				x2 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	preX1 -= x1;
	preX2 = x2 - preX2;
	preY1 -= y1;
	preY2 = y2 - preY2;

	if ( preX1 > preX2 ) {
		i0 = x1 + minX;
	} else if ( preX1 < preX2 ){
		i0 = x2 + minX - (box.w - 1);
	}

	if (  preY1 > preY2) {
		j0 = y1 + minY;
	} else if ( preY1 < preY2 ) {
		j0 = y2 + minY - (box.h - 1);
	}

	obox->x = i0;
	obox->y = j0;
	// printf("y1:%d y2:%d\n", y1, y2);
}




/* レベル1用 */
void my_tracking_level1( int frameID, unsigned char *image, int width, int height, const bounding_box *box0 )
{
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

		getImage(image, obox, width, height);
	}
	else
	{
		/* 前回の物体位置を基準にして現在の物体位置を探索 */
		setXY(image, &obox, width, height);
		// printf("%d %d\n",obox.x, obox.y );

		/* 移動物体を囲む矩形を保存 */
		set_result( frameID, obox );

		/* 現在フレームの探索範囲を確保 */
		getImage(image, obox, width, height);
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
