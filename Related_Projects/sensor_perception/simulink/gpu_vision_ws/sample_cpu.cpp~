#include <opencv2/opencv.hpp>

using namespace cv;

int main() {
	int i = 0;

	Mat src = imread("sample.jpg", 0);
	if (!src.data) exit(1);

	Mat dst;
	for( i = 0; i < 500; i ++){ 
	
	bilateralFilter(src, dst, -1, 50, 7);
	Canny(dst, dst, 35, 200, 3);
	imwrite("out.png", dst);
	}

	return 0;

}
