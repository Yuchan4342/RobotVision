#include <stdio.h>
#include <ctype.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <curses.h>
#include "get_contour.h"

#define PORT "/dev/tty.usbmodem1A1231" //適宜変更のこと
#include "serial2016.h"

#define CAMERA_CENTER_H 91  //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define CAMERA_CENTER_V 94  //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define MOTOR_DEFAULT_L 122 //左モータのデフォルト値（キャリブレーションに利用）
#define MOTOR_DEFAULT_R 135 //右モータのデフォルト値（キャリブレーションに利用）
#define CAMERA_INIT_V 62    //カメラサーボの垂直方向初期値
#define CAMERA_INIT_H 91    //カメラサーボの水平方向初期値

#define PER_SECOND 16.04   //motor(100,100)による秒速
#define ANG_V 55.05        // motor(100,156)のときの角速度(単位 °/sec)

#define CM_PER_PXS_Y 0.385     //1pxsあたりの実際の距離[cm]：y方向
#define CM_PER_PXS_X 0.444     //1pxsあたりの実際の距離[cm]：x方向
#define DIST_OUT_OF_RANGE 40.5 //カメラ範囲外からロボットまでの距離[cm]

#define SEARCHING_FOR_MARKER 0 // 探している時
#define FOUND_MARKER 1 	       // 探している時
#define ROTATE_STATE1 2        // 目標の方向に向いた時、回転
#define FORWARD_STATE1 3       // 前進その1
#define ROTATE_STATE2 4        // 90°回転
#define FORWARD_STATE2 5       // 前進その2
#define AIMING_TARGET 6        // 的に狙いをしぼる
#define FORWARD_STATE3 7       // 前進その3
#define END_STATE 8            // 目標の真上に到達したとき

#define CASE_1 1 //長方形の状態が「左上-右下」の場合
#define CASE_2 2 //長方形の状態が「真正面」の場合
#define CASE_3 3 //長方形の状態が「左下-右上」の場合
#define CASE_4 4 //長方形の状態が「真横」の場合

////////////////////////////////////////////////////////////////////////////////
#define RGB 3
#define R 0 //R：赤
#define G 1 //G：緑
#define B 2 //B：青

//各色HSVの最大・最小を要素とする構造体
typedef struct colorHSV {
	uchar minH, maxH;//色相 H の最小・最大
	uchar minS, maxS;//鮮やかさ S の最小・最大
	uchar minV, maxV;//明度 V の最小・最大
} colorHSV;
////////////////////////////////////////////////////////////////////////////////

double calcDistanceLA(int y);
double calcDistanceLB(double A, double deg, int LR);
void move(double distance);
void rotate(double deg);
int judgeCase(contourInfo topContoursInfo[]);
void on_mouse(int event, int x, int y, int flags, void *param);

int main(int argc, char **argv)
{
	CvCapture *capture = NULL; // カメラキャプチャ用の構造体
	IplImage *frame;      // キャプチャ画像 (RGB)
	IplImage *frameHSV;   // キャプチャ画像 (HSV)
	IplImage* framePT;    // 透視変換画像 (RGB)
	IplImage* framePTHSV; // 透視変換画像 (HSV)
	IplImage* frameGray = NULL; // キャプチャ画像 (グレー) Hough変換用
	CvMemStorage *storage;
	CvSeq *circles = NULL;

	IplImage* mask;      // 指定値によるmask (１チャネル)
	IplImage* contour;   // GetLargestContour() の結果
	IplImage** frames[] = {&frame, &frameHSV};
	IplImage** framesPT[] = {&framePT, &framePTHSV};
	contourInfo topContoursInfo[CONTOURS];
	int i, key, state = SEARCHING_FOR_MARKER; // state: 状態を表す変数
	float *p = NULL;
	int colors[RGB] = {R, G, B}, ci = 0; //ここで倒す順番の指定をする

	init();

	// 実習項目5.0で計測したモーターの中央値をmotor_onに、サーボの中央値をcamera_onにそれぞれセットする
	motor_on(MOTOR_DEFAULT_L, MOTOR_DEFAULT_R); // モーター静止パルス幅のキャリブレーション
	camera_on(CAMERA_CENTER_V, CAMERA_CENTER_H);   // カメラアングルキャリブレーション

	camera_horizontal(CAMERA_INIT_H); // 水平方向のカメラ角度を初期値に
	camera_vertical(CAMERA_INIT_V); // 垂直方向のカメラ角度を初期値に

	////////////////////////////////////////////////////////////////////////////////
	// 各色のHSV色．各自チューニングすること
	colorHSV rgb_HSV[RGB];
	//R
	rgb_HSV[R].minH = 110, rgb_HSV[R].maxH = 140;
	rgb_HSV[R].minS = 100, rgb_HSV[R].maxS = 230;
	rgb_HSV[R].minV = 100, rgb_HSV[R].maxV = 255;
	//G
	rgb_HSV[G].minH = 30, rgb_HSV[G].maxH = 50;
	rgb_HSV[G].minS = 70, rgb_HSV[G].maxS = 200;
	rgb_HSV[G].minV = 50, rgb_HSV[G].maxV = 255;
	//B
	rgb_HSV[B].minH = 0, rgb_HSV[B].maxH = 20;
	rgb_HSV[B].minS = 70, rgb_HSV[B].maxS = 220;
	rgb_HSV[B].minV = 50, rgb_HSV[B].maxV = 255;

	////////////////////////////////////////////////////////////////////////////////
	/*
	// 青系のHSV色．各自チューニングすること
	uchar minH = 0, maxH = 20;
	uchar minS = 70, maxS = 220;
	uchar minV =  50, maxV = 255;
	*/
	CvMat *map_matrix;
	CvPoint2D32f src_pnt[4], dst_pnt[4];

	src_pnt[0] = cvPoint2D32f(181.0, 199.0);
	src_pnt[1] = cvPoint2D32f(110.5, 199.0);
	src_pnt[2] = cvPoint2D32f(104.7, 240.0);
	src_pnt[3] = cvPoint2D32f(184.2, 240.0);
	dst_pnt[0] = cvPoint2D32f(132.5, 240.0);
	dst_pnt[1] = cvPoint2D32f(107.5, 240.0);
	dst_pnt[2] = cvPoint2D32f(107.5, 260.0);
	dst_pnt[3] = cvPoint2D32f(132.5, 260.0);
	map_matrix = cvCreateMat (3, 3, CV_32FC1);
	cvGetPerspectiveTransform (src_pnt, dst_pnt, map_matrix);

	// Initialize Camera
	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
	if (capture == NULL) {
		printf("not find camera\n");
		return -1;
	}
	// 解析速度向上のために画像サイズを下げる
	cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
	cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

	frame = cvQueryFrame(capture);
	frameHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	framePT = cvCreateImage(cvSize(240, 270), IPL_DEPTH_8U, 3);
	framePTHSV = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 3);
	frameGray = cvCreateImage (cvGetSize(frame), IPL_DEPTH_8U, 1);
	mask = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 1);
	contour = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 3);

	cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("contour", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("src", 60, 480);
	cvMoveWindow("dst", 380, 480);
	cvMoveWindow("contour", 700, 480);
	cvSetMouseCallback("src", on_mouse, (void *)frames);
	cvSetMouseCallback("dst", on_mouse, (void *)framesPT);
	cvSetMouseCallback("contour", on_mouse, (void *)framesPT);

	while (1) {
		// カメラからの入力画像1フレームをframeに格納
		frame = cvQueryFrame(capture);
		cvCvtColor(frame, frameHSV, CV_RGB2HSV);
		cvCvtColor(frame, frameGray, CV_RGB2GRAY);

		// Hough変換のための前処理
		cvSmooth (frameGray, frameGray, CV_GAUSSIAN, 11, 0, 0, 0);
		storage = cvCreateMemStorage (0);

		// Hough変換による円の検出と検出した円の描画
		circles = cvHoughCircles (frameGray, storage, CV_HOUGH_GRADIENT,
		                          1, 3.0, 20.0, 70.0, 10,
		                          MAX (frameGray->width, frameGray->height)); //円の検出
		for (i = 0; i < MIN (3, circles->total); i++) {
			p = (float *) cvGetSeqElem (circles, i);
			cvCircle (frame, cvPoint (cvRound (p[0]), cvRound (p[1])),
			          3, CV_RGB (0, 255, 0), -1, 8, 0); //円の描画
			cvCircle (frame, cvPoint (cvRound (p[0]), cvRound (p[1])),
			          cvRound (p[2]), CV_RGB (255, 0, 0), 6 - 2 * i, 8, 0); //円の描画
			printf("( x , y ) : ( %d , %d )\n", cvRound (p[0]), cvRound (p[1])); //検出した円の中心座標(x,y)を表示
		}

		// 透視変換
		cvWarpPerspective (frame, framePT, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (100));

		// ウィンドウに表示
		cvCvtColor(framePT, framePTHSV, CV_RGB2HSV);
		cvShowImage("src", frame);
		cvShowImage("dst", framePT);

		////////////////////////////////////////////////////////////////////////////////
		// 指定した色空間内の色(赤、青、緑)のみマスクする
		// GetMaskHSV(framePT, mask, minH, maxH, minS, maxS, minV, maxV);
		// color = B; //ここでRGBの指定ができるようにした
		GetMaskHSV(framePT, mask, rgb_HSV[colors[ci]].minH, rgb_HSV[colors[ci]].maxH, rgb_HSV[colors[ci]].minS, rgb_HSV[colors[ci]].maxS, rgb_HSV[colors[ci]].minV, rgb_HSV[colors[ci]].maxV);
		////////////////////////////////////////////////////////////////////////////////

		GetLargestContour(framePT, mask, contour, topContoursInfo);
		cvShowImage("contour", contour);
		key = cvWaitKey(1);
		////
		int x, y, angle, Case;
		double theta; //FORWARD_STATE1で回転する角度
		double LA, LB;
		CvBox2D oblique;
		switch (state) {
		case SEARCHING_FOR_MARKER: // マーカーを探しているとき
			if (topContoursInfo[0].area > 600) // マーカーが見つかった場合(長方形のサイズ850~920)
			{
				motor(128, 128); // 回転停止
				state = FOUND_MARKER;
			}
			else { // マーカーが見つからない場合
				motor(100, 156); // 右(時計)周りに回転
			}
			break;
		case FOUND_MARKER: // マーカーが画面内に入ったとき, 正面(カメラの中央)に入るように向きを変える
			oblique = topContoursInfo[0].oblique; // 認識した物体を囲む長方形
			x = oblique.center.x;                 // 認識した物体の画面内のx座標(0~239)
			y = oblique.center.y;                 // 認識した物体の画面内のy座標(0~269)
			if (x < 110) motor(138, 118);         // 長方形が左にあるとき左に回転
			else if (x > 130) motor(118, 138);    // 長方形が右にあるとき右に回転
			else //おおよそ正面にとらえた時
				state = ROTATE_STATE1;
			break;
		case ROTATE_STATE1: // 条件に応じて回転
			oblique = topContoursInfo[0].oblique; // 認識した物体を囲む長方形
			angle = oblique.angle;
			y = oblique.center.y;                 // 認識した物体の画面内のy座標(0~269)
			Case = judgeCase(topContoursInfo);
			LA = calcDistanceLA(y); //距離Aを計算
			switch (Case) {
			case CASE_1: //左上-右下
				LB = calcDistanceLB(LA, angle, Case); //距離Bを計算
				theta = angle;//回転する角度を計算
				break;
			case CASE_2: //真正面
				LB = 0;
				theta = 0;
				break;
			case CASE_3: //左下-右上
				LB = calcDistanceLB(LA, angle, Case); //距離Bを計算
				theta = 90 - abs(angle);//回転する角度を計算
				break;
			case CASE_4: //真横
				LB = LA * 1.4142; //距離Bを計算 Aの1/sin45°=(√2)倍
				// B = calcDistanceB(A, angle, Case); //距離Bを計算
				theta = 45; //真横なのでとりあえず右に45度回転させることとする
				break;
			}
			rotate(theta); //その場でthetaだけ回転
			state = FORWARD_STATE1;
			break;
		case FORWARD_STATE1: // 前進その1
			oblique = topContoursInfo[0].oblique; // 認識した物体を囲む長方形
			switch (Case) {
			case CASE_1: // 左上-右下のとき
			case CASE_3: // 左下-右上のとき
			case CASE_4: // 真横のとき
				move(LB);
				break;
			}
			state = ROTATE_STATE2;
			break;
		case ROTATE_STATE2: // 回転
			switch (Case) {
			case CASE_1: // 左上-右下のとき
				rotate(90);
				break;
			case CASE_3: // 右上-左下のとき
				rotate(-90);
				break;
			case CASE_4: // 真横のとき
				rotate(-135);
				break;
			}
			state = FORWARD_STATE2;
			break;
		case FORWARD_STATE2: // 前進その2
			oblique = topContoursInfo[0].oblique; // 認識した物体を囲む長方形
			x = oblique.center.x;   // 認識した物体の画面内のx座標(0~239)
			y = oblique.center.y;   // 認識した物体の画面内のy座標(0~269)
			LA = calcDistanceLA(y);	// 距離Aを計算
			move(LA - 15);
			state = AIMING_TARGET;
			break;
		case AIMING_TARGET: // 的の狙いをしぼる
			x = cvRound(p[0]);
			y = cvRound(p[1]);
			if (x < 155) motor(138, 118);      // 長方形が左にあるとき左に回転
			else if (x > 165) motor(118, 138); // 長方形が右にあるとき右に回転
			else //おおよそ正面にとらえた時
				state = FORWARD_STATE3;
			break;
		case FORWARD_STATE3: // 前進その3
			move(15);
			state = END_STATE;
			break;
		case END_STATE:
			motor_stop(); // 停止
			ci++;
			if (ci < RGB) state = SEARCHING_FOR_MARKER;
			break;
		}
		////
		if (key == 'q') break;
	}

	finalize();
	cvDestroyWindow("src");
	cvDestroyWindow("contour");
	cvReleaseImage(&frameHSV);
	cvReleaseImage(&framePTHSV);
	cvReleaseImage(&mask);
	cvReleaseImage(&contour);
	cvReleaseCapture(&capture);
	return 0;
}

//ロボットから標的までの直線距離LA[cm]を計算する関数
double calcDistanceLA(int y) {
	return (269 - y) * CM_PER_PXS_Y + DIST_OUT_OF_RANGE;
}

//ロボットの位置から長方形の長辺の垂直二等分上に引いた垂線の距離B[cm]を計算する関数
double calcDistanceLB(double LA, double deg, int Case) {
	double rad = fabs(deg) * M_PI / 180.0;
	if (Case == CASE_1) { //長方形が左上-右下状態の場合
		return LA * cos(rad);
	} else if (Case == CASE_3) { //長方形が左下-右上状態の場合
		return LA * sin(rad);
	}
	return 0; // その他の場合は0を返す
}

//指定した距離[cm]だけロボットを動かす関数
void move(double distance) {
	double move_time = distance / PER_SECOND * 1000000;//移動時間[us]の計算
	motor(100, 100); //基本速度で移動
	usleep(move_time);//計算した移動時間分だけスリープ
	motor_stop(); //停止
}

// 指定した角度deg°だけロボットをその場で右に回転する関数(引数は度数法表示で)
void rotate(double deg) {
	double move_time;
	move_time = fabs(deg / ANG_V);
	if (deg >= 0) motor(100, 156);
	else motor(156, 100);
	usleep(1000000 * move_time);
	motor_stop(); // 停止
}

//長方形のCASEを判定する関数
int judgeCase(contourInfo topContoursInfo[]) {
	int Case = 0;
	double yellowBox_width = topContoursInfo[0].oblique.size.width; //黄色の長方形のwidth
	double yellowBox_height = topContoursInfo[0].oblique.size.height; //黄色の長方形のheight
	double yellowBox_angle = topContoursInfo[0].oblique.angle; //黄色の長方形のheight
	double greenBox_width = topContoursInfo[0].horizontal.width; //緑色の長方形のwidth
	double greenBox_height = topContoursInfo[0].horizontal.height; //緑色の長方形のheight

	//条件分岐を書く
	if ((-5 <= yellowBox_angle && yellowBox_angle <= 0) || (-90 <= yellowBox_angle && yellowBox_angle <= -85)) { //真横 or 真正面
		if (greenBox_width > greenBox_height) {
			Case = CASE_2; //CASE_2(真正面)
		} else {
			Case = CASE_4; //CASE_4(真横)
		}
	} else {//左上-右下 or 左下-右上
		if (yellowBox_width < yellowBox_height) {
			Case = CASE_1; //CASE_1(左上-右下)
		} else {
			Case = CASE_3; //CASE_3(左下-右上)
		}
	}
	return Case;
}

void on_mouse(int event, int x, int y, int flags, void *frames)
{
	CvScalar BGR, HSV;
	if (event == CV_EVENT_MOUSEMOVE) {
		IplImage* rgb_image = *(((IplImage***)frames)[0]);
		IplImage* hsv_image = *(((IplImage***)frames)[1]);
		if (y < rgb_image->height && x < rgb_image->width &&
		        y < hsv_image->height && x < hsv_image->width)
		{
			BGR = cvGet2D(rgb_image, y, x);
			HSV = cvGet2D(hsv_image, y, x);
			printf("(%3d,%3d): RGB=(%3.0f,%3.0f,%3.0f) HSV=(%3.0f,%3.0f,%3.0f)\n",
			       x, y, BGR.val[2], BGR.val[1], BGR.val[0],
			       HSV.val[0], HSV.val[1], HSV.val[2]);
		}
	}
}
