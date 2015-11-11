#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "alcon2010.h"

#define SEARCH_RANGE 50	/* 移動物体の探索範囲 */
#define MISSRANGE 10		/* 探索ミスカウントの上限 */

#define PIX(img, x, y, w, i) ( img[( (x) + (y) * (w) ) * 3 + (i)] )
#define MINTEST(val, min) ( ((val) < (min)) ? (min) : (val) )
#define MAXTEST(val, max) ( ((max) < (val)) ? (max) : (val) )

typedef struct {
	int minX;
	int maxX;
	int minY;
	int maxY;
}BoundingXY;

bounding_box obox;		/* 前フレームにおける移動物体を囲む矩形 */
unsigned char *_preImg = NULL;	/* 前フレームの探索範囲の画像 */


double calcSSD(const unsigned char *srcImg, const unsigned char *destImg, int x, int y, int width);
void getBoundingXY(BoundingXY box, int *x1, int *x2, int *y1, int *y2);
void setBoundingXY(BoundingXY *box, int x1, int x2, int y1, int y2);
double sobelFilter(const unsigned char *image, int x, int y, int width);



void getImage(const unsigned char *srcImg, const bounding_box box, int width, int height)
{
	int minX, minY;
	int destWidth, destHeight;
	int x, y;

	minX = box.x - SEARCH_RANGE;
	minY = box.y - SEARCH_RANGE;
	destWidth = box.w + SEARCH_RANGE * 2;
	destHeight = box.h + SEARCH_RANGE * 2;

	if ( _preImg == NULL ) {
		_preImg = (unsigned char *)malloc( destWidth * destHeight * 3 * sizeof(unsigned char) );
	}

	/* 探索範囲の画像確保 */
	for ( y = 0; y < destHeight; y++ ) {
		if ( y + minY >= height || y + minY < 0 ) continue;

		for ( x = 0; x < destWidth; x++ ) {
			if ( x + minX >= width || x + minX < 0 ) continue;

			PIX(_preImg, x, y, destWidth, 0) = PIX(srcImg, x + minX, y + minY, width, 0);
			PIX(_preImg, x, y, destWidth, 1) = PIX(srcImg, x + minX, y + minY, width, 1);
			PIX(_preImg, x, y, destWidth, 2) = PIX(srcImg, x + minX, y + minY, width, 2);
		}
	}
}

void setXY(const unsigned char *image, bounding_box *obox, int width, int height)
{
	int e, err, merr;
	int minX, minY;
	int serachWidth, serachHeight;
	int i0, j0;
	int preX1, preX2, preY1, preY2;
	int x1, x2, y1, y2; // 変化した位置のx方向y方向の最小位置（1）と最大位置（2）を格納
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

	/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索
	上下左右それぞれの方向に走査していく */

	// 上方向
	missCount = 0;
	for ( n = y1; n >= 0; n-- ) {
		if ( n + minY >= height || n + minY < 0 ) continue;

		for ( m = 0; m < serachWidth; m++ ) {
			if ( m + minX >= width || m + minX < 0 ) continue;
			err = 0;

			e = PIX(_preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				y1 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE) break;
	}

	// 下方向
	missCount = 0;
	for ( n = y2; n < serachHeight; n++ ) {
		if ( n + minY >= height || n + minY < 0 ) continue;

		for ( m = 0; m < serachWidth; m++ ) {
			if ( m + minX >= width || m + minX < 0 ) continue;
			err = 0;

			e = PIX(_preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				y2 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	// 左方向
	missCount = 0;
	for ( m = x1; m >= 0; m-- ) {
		if ( m + minX >= width || m + minX < 0 ) continue;

		for ( n = y1; n <= y2; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;
			err = 0;

			e = PIX(_preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				x1 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	// 右方向
	missCount = 0;
	for ( m = x2; m < serachWidth; m++ ) {
		if ( m + minX >= width || m + minX < 0 ) continue;

		for ( n = y1; n <= y2; n++ ) {
			if ( n + minY >= height || n + minY < 0 ) continue;
			err = 0;

			e = PIX(_preImg, m, n, serachWidth, 0) - PIX(image, m + minX, n + minY, width, 0); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 1) - PIX(image, m + minX, n + minY, width, 1); err += e * e;
			e = PIX(_preImg, m, n, serachWidth, 2) - PIX(image, m + minX, n + minY, width, 2); err += e * e;
			if ( err > merr ) {
				x2 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	// 移動物体の幅や高さと変化したそれぞれの位置をもとに移動物体の位置を特定
	// x方向とy方向それぞれ変化の大きい方の計算を行う
	preX1 -= x1;
	preX2 = x2 - preX2;
	preY1 -= y1;
	preY2 = y2 - preY2;

	if ( preX1 > preX2 ) {
		i0 = x1 + minX;
	} else if ( preX1 < preX2 ){
		i0 = x2 + minX - box.w;
	}

	if (  preY1 > preY2) {
		j0 = y1 + minY;
	} else if ( preY1 < preY2 ) {
		j0 = y2 + minY - box.h;
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



bounding_box setSearchBox(const bounding_box box, int width, int height)
{
	bounding_box searchBox = box;
	searchBox.x -= SEARCH_RANGE;
	searchBox.y -= SEARCH_RANGE;
	searchBox.w += SEARCH_RANGE * 2;
	searchBox.h += SEARCH_RANGE * 2;

	searchBox.x = MINTEST(searchBox.x, 0);
	searchBox.y = MINTEST(searchBox.y, 0);

	if ( (searchBox.x + searchBox.w) > width  ) {
		searchBox.w -= (searchBox.x + searchBox.w) - width;
	}

	if ( (searchBox.y + searchBox.h) > height  ) {
		searchBox.h -= (searchBox.y + searchBox.h) - height;
	}

	return searchBox;
}

void setPreImg(const unsigned char *srcImg, bounding_box box, int width, int height)
{
	int x, y;
	int maxX, maxY;

	// 探索範囲の領域確保
	if ( _preImg == NULL )  {
		_preImg = (unsigned char *)malloc( box.w * box.h * 3 * sizeof(unsigned char) );
	}

	/* 探索範囲の画像確保 */
	maxX = box.x + box.w;
	maxY = box.y + box.h;
	for ( y = box.y; y < maxY; y++ ) {
		for ( x = box.x; x < maxX; x++ ) {
			PIX(_preImg, x, y, width, 0) = PIX(srcImg, x, y, width, 0);
			PIX(_preImg, x, y, width, 1) = PIX(srcImg, x, y, width, 1);
			PIX(_preImg, x, y, width, 2) = PIX(srcImg, x, y, width, 2);
		}
	}
}

BoundingXY searchOutside(const unsigned char *image, bounding_box box, BoundingXY boxXY, int width)
{
	double err, merr;
	int m, n;
	int missCount;
	int x1, x2, y1, y2;

	merr = 1000;
	getBoundingXY(boxXY, &x1, &x2, &y1, &y2);

	// 上方向
	missCount = 0;
	for ( n = y1; n >= box.y; n-- ) {
		for ( m = box.x; m < box.x + box.w; m++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				y1 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE) break;
	}

	// 下方向
	missCount = 0;
	for ( n = y2; n < box.y + box.h; n++ ) {
		for ( m = box.x; m < box.x + box.w; m++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				y2 = n;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE) break;
	}

	// 左方向
	missCount = 0;
	for ( m = x1; m >= box.x; m-- ) {
		for ( n = y1; n <= y2; n++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				x1 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	// 右方向
	missCount = 0;
	for ( m = x2; m < box.x + box.w; m++ ) {
		for ( n = y1; n <= y2; n++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				x2 = m;
				missCount = 0;
				break;
			}
		}
		missCount++;
		if ( missCount >= MISSRANGE ) break;
	}

	setBoundingXY(&boxXY, x1, x2, y1, y2);

	return boxXY;
}

BoundingXY searchInside(const unsigned char *image, bounding_box box, BoundingXY boxXY, int width)
{
	double err, merr;
	int m, n;
	int missCount;
	int x1, x2, y1, y2;

	merr = 1000;
	getBoundingXY(boxXY, &x1, &x2, &y1, &y2);

	// 下方向
	missCount = 0;
	for ( n = y1; n <= y2; n++ ) {
		for ( m = x1; m <= x2; m++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				y1 = n;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	// 上方向
	missCount = 0;
	for ( n = y2; n >= y1; n-- ) {
		for ( m = x1; m <= x2; m++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				y2 = n;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0) break;
	}

	// 右方向
	missCount = 0;
	for ( m = x1; m <= x2; m++ ) {
		for ( n = y1; n <= y2; n++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				x1 = m;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	// 左方向
	missCount = 0;
	for ( m = x2; m >= x1; m-- ) {
		for ( n = y1; n <= y2; n++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				x2 = m;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	setBoundingXY(&boxXY, x1, x2, y1, y2);

	return boxXY;
}

BoundingXY checkEdgeInside(const unsigned char *image, bounding_box box, BoundingXY boxXY, int width)
{
	int m, n;
	int missCount;
	int x1, x2, y1, y2;
	double amp;

	getBoundingXY(boxXY, &x1, &x2, &y1, &y2);

	// 右方向
	missCount = 0;
	for ( m = x1; m <= x2; m++ ) {
		for ( n = y1; n <= y2; n++ ) {
			amp = sobelFilter(image, m, n, width);
			if ( amp >= 80 ) {
				// fprintf(stderr, "%s\n", "aaaaaaa");
				x1 = m;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	// 左方向
	missCount = 0;
	for ( m = x2; m >= x1; m-- ) {
		for ( n = y1; n <= y2; n++ ) {
			amp = sobelFilter(image, m, n, width);
			if ( amp >= 80 ) {
				// fprintf(stderr, "%s\n", "aaaaaaa");
				x2 = m;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	// 上方向
	missCount = 0;
	for ( n = y1; n <= y2; n++ ) {
		for ( m = x1; m <= x2; m++ ) {
			amp = sobelFilter(image, m, n, width);
			if ( amp >= 80 ) {
				// fprintf(stderr, "%s\n", "aaaaaaa");
				y1 = n;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	// 下方向
	missCount = 0;
	for ( n = y2; n >= y1; n-- ) {
		for ( m = x1; m <= x2; m++ ) {
			amp = sobelFilter(image, m, n, width);
			if ( amp >= 80 ) {
				// fprintf(stderr, "%s\n", "aaaaaaa");
				y2 = n;
				missCount = -1;
				break;
			}
		}
		if ( missCount < 0 ) break;
	}

	setBoundingXY(&boxXY, x1, x2, y1, y2);

	return boxXY;
}

void setXY2(const unsigned char *image, bounding_box *obox, int width, int height, int id)
{
	bounding_box box;
	bounding_box searchBox;
	BoundingXY boxXY;

	box = *obox;
	searchBox = setSearchBox(box, width, height);

	boxXY.minX = box.x;
	boxXY.maxX = box.x + box.w - 1;
	boxXY.minY = box.y;
	boxXY.maxY = box.y + box.h - 1;

	/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索
	上下左右それぞれの方向に走査していく
	外側に向けて変化している位置を特定 */
	boxXY = searchOutside(image, searchBox, boxXY, width);

	// 内側に向けて変化している位置を特定
	boxXY = searchInside(image, searchBox, boxXY, width);

	// 内側に向けてエッジ検出を行う
	boxXY = checkEdgeInside(image, searchBox, boxXY, width);

	obox->x = boxXY.minX;
	obox->y = boxXY.minY;
	obox->w = boxXY.maxX - boxXY.minX + 1;
	obox->h = boxXY.maxY - boxXY.minY + 1;
}

/* レベル2用 */
void my_tracking_level2( int frameID, unsigned char *image, int width, int height )
{
	bounding_box searchBox;
	if( frameID == 0 )
	{
		/* 最初のフレームは追跡開始位置を記憶 */
		// 001,540,132,86,80
		// 002,5,230,202,121
		// 003,415,213,215,121
		// 004,505,230,26,56
		// 005,157,256,53,48
		obox.x = 0;
		obox.y = 0;
		obox.w = width;
		obox.h = height;

		/* 移動物体を囲む矩形を設定 */
		set_result( frameID, obox );

		setPreImg(image, obox, width, height);
	}
	else
	{
		/* 前回の物体位置を基準にして現在の物体位置を探索 */
		setXY2(image, &obox, width, height, frameID);
		// printf("%d %d\n",obox.x, obox.y );

		/* 移動物体を囲む矩形を保存 */
		set_result( frameID, obox );

		searchBox = setSearchBox(obox, width, height);
		/* 現在フレームの探索範囲を確保 */
		setPreImg(image, searchBox, width, height);
	}
}

/* レベル3用 */
void my_tracking_level3( int frameID, unsigned char *image, int width, int height )
{
}









double calcSSD(const unsigned char *srcImg, const unsigned char *destImg, int x, int y, int width)
{
	double e, err;

	err = 0.0;
	e = PIX(srcImg, x, y, width, 0) - PIX(destImg, x, y, width, 0);
	err += e * e;
	e = PIX(srcImg, x, y, width, 1) - PIX(destImg, x, y, width, 1);
	err += e * e;
	e = PIX(srcImg, x, y, width, 2) - PIX(destImg, x, y, width, 2);
	err += e * e;

	return err;
}

void getBoundingXY(BoundingXY box, int *x1, int *x2, int *y1, int *y2)
{
	*x1 = box.minX;
	*x2 = box.maxX;
	*y1 = box.minY;
	*y2 = box.maxY;
}

void setBoundingXY(BoundingXY *box, int x1, int x2, int y1, int y2)
{
	box->minX = x1;
	box->maxX = x2;
	box->minY = y1;
	box->maxY = y2;
}

double sobelFilter(const unsigned char *image, int x, int y, int width)
{
	static int sobelX[3][3] = {
		{-1,0,1},
		{-2,0,2},
		{-1,0,1}
	};
	static int sobelY[3][3] = {
		{-1,-2,-1},
		{0,0,0},
		{1,2,1}
	};
	int val;
	int m, n;
	double deltaX, deltaY;

	deltaX = deltaY = 0.0;
	val = 0;
	for ( n = -1; n < 2; n++ ) {
		for ( m = -1; m < 2; m++ ) {
			val = 	PIX(image, m + x, n + y, width, 0) +
				PIX(image, m + x, n + y, width, 1) +
				PIX(image, m + x, n + y, width, 2);
			val /= 3;

			deltaX += val * sobelX[n+1][m+1];
			deltaY += val * sobelY[n+1][m+1];
		}
	}

	return (fabs(deltaX) + fabs(deltaY)) / 2;
}
