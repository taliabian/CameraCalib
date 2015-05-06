#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <windows.h>

#ifdef WIN32
#include <io.h>
#else 
#endif

using namespace std;

int n_boards = 0;
int board_w;
int board_h;

void ReadDirectory( const string& dirName, const string fileFea, vector<string>& filenames );

int main( int argc, char* argv[] )
{
	cout << "enter the width and height of board: "<<endl;
	cout << "w: ";
	cin >> board_w;
	cout << "h: ";
	cin >> board_h;

	vector<string> filenames;
	ReadDirectory(".\\image", ".jpg", filenames);
	char savename[200];
	
	char num[10];
	n_boards = filenames.size();
	int board_n = board_w*board_h;
	CvSize board_sz = cvSize( board_w, board_h );
	
	CvMat* image_points = cvCreateMat( n_boards*board_n, 2, CV_32FC1 ); // 特征点图像坐标
	CvMat* object_points = cvCreateMat( n_boards*board_n, 3, CV_32FC1 );// 特征点世界坐标
	CvMat* point_counts = cvCreateMat( n_boards, 1, CV_32FC1 ); // 
	CvMat* instrinsic_matrix = cvCreateMat( 3, 3, CV_32FC1 ); // 内参矩阵
	CvMat* distortion_coeffs = cvCreateMat( 5, 1, CV_32FC1 ); // 畸变系数

	CvPoint2D32f *corners = new CvPoint2D32f[ board_n];
	int corner_count;
	int successes = 0;
	int step;
	
	
	IplImage *image = cvLoadImage(filenames[0].c_str(), -1);
	if( image == NULL)
		cout<<"cannot read images"<<endl;
	IplImage *gray_image = cvCreateImage( cvGetSize(image), 8, 1); // 灰度单通道图像

	while( successes < n_boards ) //成功检测到角点信息
	{
		int found = cvFindChessboardCorners( image, board_sz, corners, &corner_count,
											 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );// 粗检测
		cvCvtColor( image, gray_image, CV_RGB2GRAY );
		cvFindCornerSubPix( gray_image, corners, corner_count, cvSize(11,11), cvSize(-1,-1), cvTermCriteria(
							CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)); // 检测角点(亚像素)
		cvDrawChessboardCorners( image, board_sz, corners, corner_count, found ); // 绘角点
		cvShowImage( filenames[successes].c_str() , image );// 显示

		if( corner_count == board_n )// 检测出角点数=实际角点数
		{
			savename[0] = '\0';
			GetCurrentDirectory(200,savename); 
			strcat( savename, "\\reslut\\cal_");
			strcat( savename, itoa(successes,num,10));
			strcat( savename, ".jpg");
			cvSaveImage(savename,image);// 保存

			step = successes*board_n;
			for( int i=step, j=0; j<board_n; i++, j++){
				CV_MAT_ELEM( *image_points, float, i,0 ) = corners[j].x;
				CV_MAT_ELEM( *image_points, float, i,1 ) = corners[j].y;
				CV_MAT_ELEM( *object_points, float, i,0 ) = j/board_w;
				CV_MAT_ELEM( *object_points, float, i,1 ) = j%board_w;
				CV_MAT_ELEM( *object_points, float, i,2 ) = 0.0f;
			}
			CV_MAT_ELEM( *point_counts, int, successes ,0 ) = board_n;
			successes++;
		}

		int c = cvWaitKey(0);
		if( c == 'y' &&  successes!= n_boards )
		{
			cvDestroyWindow(filenames[successes-1].c_str()); 
			image = cvLoadImage(filenames[successes].c_str(), -1);
		}
		else if( c==27 )
			return 0;
	}
	CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
	CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
	CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );

	for( int i=0; i<successes*board_n; ++i)
	{
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0);
		CV_MAT_ELEM( *image_points2, float, i,1)  = CV_MAT_ELEM( *image_points, float, i,1 );
		CV_MAT_ELEM( *object_points2, float, i,0 ) = CV_MAT_ELEM( *object_points, float, i,0 );
		CV_MAT_ELEM( *object_points2, float, i,1 ) = CV_MAT_ELEM( *object_points, float, i,1 );
		CV_MAT_ELEM( *object_points2, float, i,2 ) = CV_MAT_ELEM( *object_points, float, i,2 );
	}
	for( int i=0; i<successes; ++i)
	{
		CV_MAT_ELEM( *point_counts2, int, i, 0) = CV_MAT_ELEM( *point_counts, int, i, 0);
	}
	cvReleaseMat( &object_points);
	cvReleaseMat( &image_points);
	cvReleaseMat( &point_counts);

	// 标定
	CV_MAT_ELEM( *instrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *instrinsic_matrix, float, 1, 1 ) = 1.0f;

	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize(image), 
						instrinsic_matrix, distortion_coeffs, 
						NULL, NULL, 0);
	cvSave("Instrinsics.xml", instrinsic_matrix );
	cvSave("Distortion.xml", distortion_coeffs );

	//畸变矫正
	CvMat *instrinsic = (CvMat*)cvLoad("Instrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
	IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1);
	cvInitUndistortMap( instrinsic, distortion, mapx, mapy ); //畸变矫正图像x,y
	cvNamedWindow( "Undistort");
	successes = 0;
	image = cvLoadImage(filenames[successes].c_str(), -1);
	while( successes < n_boards ){
		IplImage* t = cvCloneImage(image);//原始
		cvShowImage("Calibration", image);
		cvRemap( t, image, mapx, mapy );//畸变矫正后
		cvShowImage("Undistort", image);

		savename[0] = '\0';
		GetCurrentDirectory(200,savename); 
		strcat( savename, "\\reslut\\undis_");
		strcat( savename, itoa(successes,num,10));
		strcat( savename, ".jpg");
		cvSaveImage(savename,image);// 保存

		int c = cvWaitKey(0);
		successes++;
		if( c == 'y' &&  successes!= n_boards )
		{
			image = cvLoadImage(filenames[successes].c_str(), -1);
		}
		else if( c==27 )
			return 0;
	}
	system("pause");
	return 0;
}

/// 读取文件
void ReadDirectory( const string& dirName, const string fileFea, vector<string>& filenames )
{
	filenames.clear();
#ifdef WIN32
	struct _finddata_t s_file;
	string str = dirName + "\\*" + fileFea;
	intptr_t h_file = _findfirst( str.c_str(), &s_file);
	if( h_file != static_cast<intptr_t>(-1.0) )
	{
		do{
			filenames.push_back( dirName + "\\" + (string)s_file.name);
		}
		while( _findnext( h_file, &s_file )==0 );
	}
	_findclose( h_file );
#else
#endif
	sort( filenames.begin(), filenames.end() );
}