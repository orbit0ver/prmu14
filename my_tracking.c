#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "alcon2010.h"

#define SEARCH_RANGE 50	/* 移動物体の探索範囲 */
#define MISSRANGE 10		/* 探索ミスカウントの上限 */
#define MERR 1000
#define MAMP 55

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
bounding_box _searchBox;


double calcSSD(const unsigned char *srcImg, const unsigned char *destImg, int x, int y, int width);
void getBoundingXY(BoundingXY box, int *x1, int *x2, int *y1, int *y2);
void setBoundingXY(BoundingXY *box, int x1, int x2, int y1, int y2);
double sobelFilter(const unsigned char *image, int x, int y, int width, int height);
void setSearchBox(const bounding_box box, int width, int height);
void setPreImg(const unsigned char *srcImg, bounding_box box, int width, int height);
BoundingXY searchOutside(const unsigned char *image, BoundingXY boxXY, int width);
void setXY(const unsigned char *image, bounding_box *obox, int width, int height);
BoundingXY searchInside(const unsigned char *image, BoundingXY boxXY, int width, int height);
void setXY2(const unsigned char *image, bounding_box *obox, int width, int height, int id);



/* レベル1用 */
void my_tracking_level1( int frameID, unsigned char *image, int width, int height, const bounding_box *box0 )
{
	/*
		画像をBMP形式で保存する場合は次の様に記述する
		save_bmp( "hoge.bmp", image, width, height );
	*/

	if( frameID == 0 ) {
		/* 最初のフレームは追跡開始位置を記憶 */
		obox = *box0;
		set_result( frameID, obox );

		setSearchBox(obox, width, height);
		setPreImg(image, _searchBox, width, height);
	} else {
		/* 前回の物体位置を基準にして現在の物体位置を探索 */
		setXY(image, &obox, width, height);

		/* 移動物体を囲む矩形を保存 */
		set_result( frameID, obox );

		setSearchBox(obox, width, height);
		setPreImg(image, _searchBox, width, height);
	}
}

/* レベル2用 */
void my_tracking_level2( int frameID, unsigned char *image, int width, int height )
{
	if ( frameID == 0 ) {
		// 移動物体を検出するため全領域を設定
		obox.x = 0;
		obox.y = 0;
		obox.w = width;
		obox.h = height;

		/* 移動物体を囲む矩形を設定 */
		set_result( frameID, obox );

		setSearchBox(obox, width, height);
		setPreImg(image, _searchBox, width, height);
	} else {
		/* 前回の物体位置を基準にして現在の物体位置を探索 */
		setXY2(image, &obox, width, height, frameID);
		// printf("%d %d\n",obox.x, obox.y );

		/* 移動物体を囲む矩形を保存 */
		set_result( frameID, obox );

		setSearchBox(obox, width, height);
		/* 現在フレームの探索範囲を確保 */
		setPreImg(image, _searchBox, width, height);
	}
}

/* レベル3用 */
void my_tracking_level3( int frameID, unsigned char *image, int width, int height )
{
	my_tracking_level2(frameID, image, width, height);
}



/**  functions **/

/*
// 説明 //
	引数で指定した範囲の画像情報をグローバル変数_preImgに格納する

// 引数 //
	srcImg	:入力画像
	box 	:保存する画像の矩形情報（位置）
	width 	:入力画像の幅
	height 	:入力画像の高さ
*/

void setXY(const unsigned char *image, bounding_box *obox, int width, int height)
{
	int x, y;
	int subX1, subX2, subY1, subY2;
	bounding_box box;
	BoundingXY boxXY;
	BoundingXY newBoxXY;

	box = *obox;
	x = box.x;
	y = box.y;

	// 前フレームの移動物体の矩形情報保存
	boxXY.minX = box.x;
	boxXY.maxX = box.x + box.w - 1;
	boxXY.minY = box.y;
	boxXY.maxY = box.y + box.h - 1;

	/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索
	上下左右それぞれの方向に走査していく
	外側に向けて変化している位置を特定 */
	newBoxXY = searchOutside(image, boxXY, width);

	// 移動物体の幅や高さと変化したそれぞれの位置をもとに移動物体の位置を特定
	// x方向とy方向それぞれ変化の大きい方の計算を行う
	subX1 = abs(boxXY.minX - newBoxXY.minX);
	subX2 = abs(boxXY.maxX - newBoxXY.maxX);
	subY1 = abs(boxXY.minY - newBoxXY.minY);
	subY2 = abs(boxXY.maxY - newBoxXY.maxY);

	x = (subX1 > subX2) ? newBoxXY.minX : newBoxXY.maxX - box.w + 1;
	y = (subY1 > subY2) ? newBoxXY.minY : newBoxXY.maxY - box.h + 1;

	obox->x = x;
	obox->y = y;
}


void setSearchBox(const bounding_box box, int width, int height)
{
	int maxX, maxY;

	// 探索範囲の初期値設定
	_searchBox = box;
	_searchBox.x -= SEARCH_RANGE;
	_searchBox.y -= SEARCH_RANGE;
	_searchBox.w += SEARCH_RANGE * 2;
	_searchBox.h += SEARCH_RANGE * 2;

	// 探索範囲の上限下限の設定
	_searchBox.x = MINTEST(_searchBox.x, 0);
	_searchBox.y = MINTEST(_searchBox.y, 0);

	maxX = _searchBox.x + _searchBox.w;
	if ( maxX > width  ) {
		_searchBox.w -= maxX - width;
	}

	maxY = _searchBox.y + _searchBox.h;
	if ( maxY > height  ) {
		_searchBox.h -= maxY - height;
	}
}

void setPreImg(const unsigned char *srcImg, bounding_box box, int width, int height)
{
	int x, y;
	int maxX, maxY;

	// 画像の領域確保
	if ( _preImg == NULL )  {
		_preImg = (unsigned char *)malloc( width * height * 3 * sizeof(unsigned char) );
	}

	// 探索範囲の値取得
	maxX = box.x + box.w;
	maxY = box.y + box.h;
	for ( y = box.y; y < maxY; y++ ) {
		if ( y < 0 || height <= y ) continue;

		for ( x = box.x; x < maxX; x++ ) {
			if ( x < 0 || width <= x ) continue;

			PIX(_preImg, x, y, width, 0) = PIX(srcImg, x, y, width, 0);
			PIX(_preImg, x, y, width, 1) = PIX(srcImg, x, y, width, 1);
			PIX(_preImg, x, y, width, 2) = PIX(srcImg, x, y, width, 2);
		}
	}
}

BoundingXY searchOutside(const unsigned char *image, BoundingXY boxXY, int width)
{
	double err, merr;
	int m, n;
	int missCount;
	int x1, x2, y1, y2;

	merr = MERR;
	getBoundingXY(boxXY, &x1, &x2, &y1, &y2);

	// 上方向
	missCount = 0;
	for ( n = y1; n >= _searchBox.y; n-- ) {
		for ( m = _searchBox.x; m < _searchBox.x + _searchBox.w; m++ ) {
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
	for ( n = y2; n < _searchBox.y + _searchBox.h; n++ ) {
		for ( m = _searchBox.x; m < _searchBox.x + _searchBox.w; m++ ) {
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
	for ( m = x1; m >= _searchBox.x; m-- ) {
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
	for ( m = x2; m < _searchBox.x + _searchBox.w; m++ ) {
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

BoundingXY searchInside(const unsigned char *image, BoundingXY boxXY, int width, int height)
{
	double err, merr;
	int m, n;
	int missCount;
	int x1, x2, y1, y2;
	double amp;

	merr = MERR;
	getBoundingXY(boxXY, &x1, &x2, &y1, &y2);

	// 下方向
	missCount = 0;
	for ( n = y1; n <= y2; n++ ) {
		for ( m = x1; m <= x2; m++ ) {
			err = calcSSD(_preImg, image, m, n, width);
			if ( err > merr ) {
				amp = sobelFilter(image, m, n, width, height);
				if ( amp >= MAMP ) {
					y1 = n;
					missCount = -1;
					break;
				}
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
				amp = sobelFilter(image, m, n, width, height);
				if ( amp >= MAMP ) {
					y2 = n;
					missCount = -1;
					break;
				}
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
				amp = sobelFilter(image, m, n, width, height);
				if ( amp >= MAMP ) {
					x1 = m;
					missCount = -1;
					break;
				}
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
				amp = sobelFilter(image, m, n, width, height);
				if ( amp >= MAMP ) {
					x2 = m;
					missCount = -1;
					break;
				}
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
	BoundingXY boxXY;

	box = *obox;

	boxXY.minX = box.x;
	boxXY.maxX = box.x + box.w - 1;
	boxXY.minY = box.y;
	boxXY.maxY = box.y + box.h - 1;

	/* 前フレームと現フレームの探索範囲での変化を調べ、移動物体位置を探索
	上下左右それぞれの方向に走査していく
	外側に向けて変化している位置を特定 */
	boxXY = searchOutside(image,  boxXY, width);

	// 内側に向けて変化している位置を特定しエッジ検出を行う
	boxXY = searchInside(image, boxXY, width, height);

	obox->x = boxXY.minX;
	obox->y = boxXY.minY;
	obox->w = boxXY.maxX - boxXY.minX + 1;
	obox->h = boxXY.maxY - boxXY.minY + 1;
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

double sobelFilter(const unsigned char *image, int x, int y, int width, int height)
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
	int tmpX, tmpY;

	deltaX = deltaY = 0.0;
	val = 0;
	for ( n = -1; n < 2; n++ ) {
		for ( m = -1; m < 2; m++ ) {
			if ( (m + x) >= width || (n + y) >= height ) {
				tmpX = MAXTEST(m + x, width - 1);
				tmpY = MAXTEST(n + y, height - 1);
				val = 	PIX(image, tmpX, tmpY, width, 0) +
					PIX(image, tmpX, tmpY, width, 1) +
					PIX(image, tmpX, tmpY, width, 2);
			} else {
				tmpX = MINTEST(m + x, 0);
				tmpY = MINTEST(n + y, 0);
				val = 	PIX(image, tmpX, tmpY, width, 0) +
					PIX(image, tmpX, tmpY, width, 1) +
					PIX(image, tmpX, tmpY, width, 2);
			}
			val /= 3;

			deltaX += val * sobelX[n+1][m+1];
			deltaY += val * sobelY[n+1][m+1];
		}
	}

	return (fabs(deltaX) + fabs(deltaY)) / 2;
}
