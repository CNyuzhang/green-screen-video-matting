#include "sharedmatting.h"
#include <string>

using namespace std;
using namespace cv;

/****函数声明****/
Mat getmask(Mat part);
Mat getmask1(Mat part);
Mat get_trimap(Mat img);
int savevideo_init(Mat frame);
CvRect get_point(Mat out); 
void on_mouse(int event,int x,int y, int flags,void* ustc);
void on_trackbar(int trackbar);
void switch_callback(int position);
Mat compostion(Mat alpha,Mat frame,Mat background);
Mat compostion1(Mat alpha,Mat frame,Mat background);
void compostion_first(Mat part,Mat background,Mat mask_process);
Mat Same_Composition(Mat frame,Mat background);
Mat Variable_Composition(Mat frame,Mat background);
Mat Test_composition(Mat frame,Mat background);
void init(Mat frame,Mat background);
int video_init();


/****全局变量****/
int set_x=760,set_y=365;         //xmax=760  ymin=365
float proportion=1.0;
int trackbar=100;
int rows=0,cols=0,rows_p=0,cols_p=0;
int nThreshold=0;
int g_switch_value = 0;
int flag_began =0;
long nFrame=0;
CvRect rect;
char fileAddr[64] = {0};

VideoWriter writer;
VideoCapture capture;
VideoCapture captureb;
int main()
{
	Mat frame,background,part;

	if(video_init()<0)
		return -1;

	capture >> frame;
	captureb >> background;
	cout << "\n background depth=" << background.size() << "\t background type=" << background.type() << "\t background channel=" << background.channels() << endl;
	cout << "frame depth=" << frame.size() << "\t frame type=" << frame.type() << "\tframe channel=" << frame.channels() << endl;

	init(frame,background);
	resize(frame,frame,frame.size()/2);
	//imshow("frame is ",frame);
	/*****设置位置以及大小*****/
	while(capture.read(frame) && !flag_began  && captureb.read(background))
	{
		Mat test =  Test_composition(frame,background);
		imshow("the composition picture",test);
		waitKey(-1);
	}
	cvDestroyWindow("the composition picture");
	printf("now beganing to produce compostion video.......\n");

	///*****重新装载视频，前面操作后当前帧可能不是起始帧，并开始合成*****/
	capture.open("yang.mp4");
	captureb.open("cityriver_720p.mov");
	int nowframe=0;
	while(capture.read(frame) && captureb.read(background))
	{
		nowframe++;
		printf("\n now frame is %d / %d \n",nowframe,nFrame);

		Mat out = Variable_Composition(frame,background);
		//Mat out = Same_Composition(frame,background);
		writer.write(out);
		//imshow("the composition picture",out);

		//waitKey(-1);
	}
	return 0;
}
	


Mat compostion(Mat alpha,Mat frame,Mat background)
{
	for(int i=set_y,ii=rows_p-1; i>set_y-rows_p;i--,ii--)
	{
		uchar *p_frame=frame.ptr<uchar>(ii);
		uchar *p_mask=alpha.ptr<uchar>(ii);
 		uchar *p_background=background.ptr<uchar>(i) + set_x*3;
	 	for(int j=set_x;j<set_x+cols_p;j++)
		{
				double w = *p_mask/255.0;

				*p_background=(*p_frame++)*w+(*p_background++)*(1-w);
				*p_background=(*p_frame++)*w+(*p_background++)*(1-w);
				*p_background=(*p_frame++)*w+(*p_background++)*(1-w);
			p_mask++;
		}
	}
	return background;
}

Mat compostion1(Mat alpha,Mat frame,Mat background)
{
	Mat out = Mat::zeros(background.size(), background.type());
	int col = alpha.cols;
	int row = alpha.rows;
	
	for(int i=0; i<row;i++)
	{
		uchar *p_frame=frame.ptr<uchar>(i);
		uchar *p_out=out.ptr<uchar>(i);
		uchar *p_mask=alpha.ptr<uchar>(i);
 		uchar *p_background=background.ptr<uchar>(i);
	 	for(int j=0;j<col;j++)
		{
				double w = *p_mask/255.0;

				*p_out++=(*p_frame++)*w+(*p_background++)*(1-w);
				*p_out++=(*p_frame++)*w+(*p_background++)*(1-w);
				*p_out++=(*p_frame++)*w+(*p_background++)*(1-w);
			p_mask++;
		}
	}
	return out;
}
void compostion_first(Mat part,Mat background,Mat mask_process)
{
	for(int i=set_y,ii=rows_p-1; i>set_y-rows_p;i--,ii--)
	{
		uchar *p_frame=part.ptr<uchar>(ii);
		uchar *p_mask=mask_process.ptr<uchar>(ii);
 		uchar *p_background=background.ptr<uchar>(i)+set_x*3;
	 	for(int j=set_x;j<set_x+cols_p;j++)
		{
			if(*p_mask == 0)	
			{
				*p_background=*(p_frame);
				*(p_background+1)=*(p_frame+1);
				*(p_background+2)=*(p_frame+2);
			}
			p_background+=3;
			p_frame+=3;
			p_mask++;
		}
	}
}


Mat Same_Composition(Mat frame,Mat background)
{
		Mat mask = getmask1(frame);
		Mat trimap = get_trimap(mask);
		imwrite("trimap/trimap1.png", trimap);
		imwrite("input/input1.png", frame);
		//imshow("trimap is ",trimap);
		//imshow("frame is ",frame);

		////开始获取alpha
		SharedMatting sm;
		sm.loadImage("input/input1.png");
		sm.loadTrimap("trimap/trimap1.png");;
		sm.solveAlpha();
		sprintf(fileAddr, "result/GT%d%d.png", 10/10, 10%10);
		sm.save(fileAddr);

		Mat alpha = imread("result/GT10.png",0);

		cout << "background depth=" << background.size() << "background, type=" << background.type() << "background channels=" << background.channels() << endl;
		resize(background,background,alpha.size());
		//imshow("background is",background);
		Mat out = compostion(alpha,frame,background);	
		return out;
}

Mat Variable_Composition(Mat frame,Mat background)
{
		resize(frame,frame,frame.size()/2);      //原图过大，只是用来缩小下
		Mat part=frame(rect);
		resize(part,part,Size(0,0),proportion,proportion,INTER_LINEAR);
		imwrite("input/input.png", part);

		//resize(part,part,Size(0,0),proportion,proportion,INTER_LINEAR);
		//cout << "part depth=" << part.size() << "\t part type=" << part.type() << "\t part channels=" << part.channels() << endl;
		//cout << "background depth=" << background.size() << "\t background, type=" << background.type() << "\t background channels=" << background.channels() << endl;
		rows_p=part.rows;
		cols_p=part.cols;
		//imshow("part is ",part);

		Mat mask = getmask(part);
		//imshow("part is ",mask);
		Mat trimap = get_trimap(mask);
		//imshow("the trimap is ",trimap);
		imwrite("trimap/trimap.png", trimap);
		
		////开始获取alpha
		SharedMatting sm;
		sm.loadImage("input/input.png");
		sm.loadTrimap("trimap/trimap.png");
		sm.solveAlpha();
		sprintf(fileAddr, "result/result.png");
		sm.save(fileAddr);

		//开始合成
		Mat alpha = imread("result/result.png",0);
		Mat out = compostion(alpha,part,background);	
		//imshow("the alpha is ",alpha);
		return out;
}
Mat Test_composition(Mat frame,Mat background)
{
		resize(frame,frame,frame.size()/2);      //原图过大，只是用来缩小下
		Mat part=frame(rect);
		resize(part,part,Size(0,0),proportion,proportion,INTER_LINEAR);
		rows_p=part.rows; 
		cols_p=part.cols;
		Mat mask = getmask1(part);
		compostion_first(part,background,mask);
		return background;
}

//用于获取trimap
Mat getmask(Mat part)
{
	Mat img,mask;
	cvtColor(part, img, COLOR_BGR2HSV);      //将frame从RGB转化为HSV

	inRange(img, Scalar(35, 103, 46), Scalar(155, 255, 255), mask); // 二值化，分别低与高的阈值

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));  //定义结构元素
	morphologyEx(mask, mask, MORPH_CLOSE, element);					// 开运算    去掉黑色前景色上的白点（去亮的）  不同参数不同处理
	//erode(mask, mask, element);										// 腐蚀 ，圆滑边界，默认执行一次，可改
	if(proportion<0.55)
	{
		blur(mask,mask,Size(3,3));
	}
	else if(proportion < 0.85)
	{
		dilate(mask, mask, element);
		blur(mask,mask,Size(4,4));
	}
	else if(proportion >1.2)
		{
		dilate(mask, mask, element);dilate(mask, mask, element);erode(mask, mask, element);dilate(mask, mask, element);dilate(mask, mask, element);
		erode(mask, mask, element);erode(mask, mask, element);
		blur(mask,mask,Size(4,4));
	
		}
	else
	{
		dilate(mask, mask, element);dilate(mask, mask, element);erode(mask, mask, element);dilate(mask, mask, element);dilate(mask, mask, element);
		erode(mask, mask, element);erode(mask, mask, element);
		blur(mask,mask,Size(6,6));
	}
	return mask;
}

//用于圈出目标
Mat getmask1(Mat part)
{
	Mat img,mask;
	cvtColor(part, img, COLOR_BGR2HSV);      //将frame从RGB转化为HSV

	inRange(img, Scalar(35, 103, 46), Scalar(155, 255, 255), mask); // 二值化，分别低与高的阈值

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));  //定义结构元素
	morphologyEx(mask, mask, MORPH_CLOSE, element);					// 开运算    去掉黑色前景色上的白点（去亮的）  不同参数不同处理
	//erode(mask, mask, element);										// 腐蚀 ，圆滑边界，去除白点，默认执行一次，可改
	
	dilate(mask, mask, element);
	erode(mask, mask, element);
	dilate(mask, mask, element);	
	
	//GaussianBlur(mask, mask, Size(5, 5), 0, 0);
	blur(mask,mask,Size(3,3));
	return mask;
}

Mat get_trimap(Mat img)
{
	Mat result=Mat::zeros(img.size(),CV_8UC1);	
	for(int i=0;i<img.rows;++i)
	{
		uchar *p_img=img.ptr<uchar>(i);
		uchar *p_result=result.ptr<uchar>(i);
		for(int j=0;j<img.cols;++j)
		{
			if(*p_img == 0)
				*p_result = 255;
			else if(*p_img == 255)
				*p_result = 0;
			else
				*p_result = 128;

			p_result++;
			p_img++;
		}
	}
	return result;
}

vector<vector<Point>> contours;                    //放循环内部出错
vector<Vec4i> hierarchy;
CvRect get_point(Mat out)                           //得到所有边缘点
{
	//参考   https://blog.csdn.net/dcrmg/article/details/51987348
	CvRect point;
	findContours(out,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);		//mode取值“CV_RETR_EXTERNAL”，method取值“CV_CHAIN_APPROX_NONE”，即只检测最外层轮廓，并且保存轮廓上所有点
																							// mode取值“CV_RETR_LIST”，method取值“CV_CHAIN_APPROX_SIMPLE”，即检测所有轮廓，但各轮廓之间彼此独立，不建立等级关系，并且仅保存轮廓上拐点信息：

	Mat imageContours=Mat::zeros(out.size(),CV_8UC1);
	Mat Contours=Mat::zeros(out.size(),CV_8UC1); 
	//Mat rect = Mat::zeros(out.size(),CV_8UC3);
	Mat rect=out;

	for(int i=0;i<contours.size();i++)
	{
		CvRect boundRect=boundingRect(contours[i]);

		//此处注释为绘制轮廓部分

		////contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数		
		//for(int j=0;j<contours[i].size();j++) 	
		//{		
		//			//绘制出contours向量内所有的像素点		
		//	Point P=Point(contours[i][j].x,contours[i][j].y);		
		//	Contours.at<uchar>(P)=255;	
		//} 	
		//		//输出hierarchy向量内容		
		//		char ch[256];		
		//		sprintf(ch,"%d",i);		
		//		string str=ch;	
		//		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl; 	
		////绘制轮廓		
		//drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);

		if(boundRect.width>100 && boundRect.height>100 && boundRect.width <1000)    //只绘制目标所在区域的框。
		{
			point.x=boundRect.x;
			point.y=boundRect.y;
			point.width=boundRect.width;
			point.height=boundRect.height;
		}
	}	
	point.x-=10;
	point.y-=20;
	point.height+=40;
	point.width+=80;
	//imshow("Contours Image",imageContours); //轮廓
	//imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
	printf("\n the rect: x=%d,y=%d,h=%d,W=%d \n",point.x,point.y,point.height,point.width);
	return point;
}

void on_mouse(int event,int x,int y, int flags,void* ustc)    //最后参数什么意思
{
	static CvPoint pre_pt =(-1,-1);
	static CvPoint cur_pt =(-1,-1);
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);

	if(event == CV_EVENT_LBUTTONDOWN)							//按下鼠标事件
	{
		if(x > (1280 - cols_p*proportion))
			set_x = 1270 - cols_p*proportion;
		else
			set_x = x;
		if(y < rows_p*proportion)
			set_y=rows_p*proportion;
		else
			set_y=y;
		printf("\n set_x is %d,\t set_y is %d \n ",x,y);
	}
	//else if(event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))   //移动鼠标
	//{
	//	printf("now posin (%d,%d)\n",x,y);
	//
	//}
}

void on_trackbar(int trackbar)
{
	proportion=(float)(trackbar/100.0);
	if(proportion < 0.3)proportion=0.3;
	else if(proportion>MIN((set_y/(rows_p*1.0)),((1280-set_x)/(cols_p*1.0))) )proportion=MAX((set_y/(rows_p*1.0)),((1280-set_x)/(cols_p*1.0)));
}

void switch_callback(int position)
{
    if (position == 0) 
	{
		flag_began =0;
        printf("now flag is 0 \n");
    }
    else 
	{
		flag_began =1;
        printf("now flag is 1 \n");
    }
}


void init(Mat frame,Mat background)
{
	resize(frame,frame,frame.size()/2);  
	Mat mask1 = getmask1(frame);
	//imshow("mask1 is ",mask1);
	savevideo_init(background);
	////注意  只能用于有闭合轮廓的图像
	rect=get_point(mask1);

	//设置滑动条与鼠标点击事件以调整大小与位置
	cvNamedWindow("the composition picture",1);
	cvSetMouseCallback("the composition picture",on_mouse,0);
	cvCreateTrackbar("proportion","the composition picture",&trackbar,300,on_trackbar);
	cvCreateTrackbar("Switch", "the composition picture", &g_switch_value, 1, switch_callback);

}




/***********************************************/
/* CV_FOURCC('P','I','M','1') = MPEG-1 codec
/* CV_FOURCC('M','J','P','G') = motion-jpeg codec
/* CV_FOURCC('M', 'P', '4', '2') = MPEG-4.2 codec
/* CV_FOURCC('D', 'I', 'V', '3') = MPEG-4.3 codec
/* CV_FOURCC('D', 'I', 'V', 'X') = MPEG-4 codec
/* CV_FOURCC('U', '2', '6', '3') = H263 codec
/* CV_FOURCC('I', '2', '6', '3') = H263I codec
/* CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec
--------------------- */
/*************************************************/

int savevideo_init(Mat frame)
{
	 bool isColor = (frame.type() == CV_8UC3);

    //--- INITIALIZE VIDEOWRITER
    int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
    double fps = 25.0;                          // framerate of the created video stream
    string filename = "./result.avi";             // name of the output video file
    writer.open(filename, codec, fps, frame.size(), isColor);
    // check if we succeeded
    if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "\n Writing videofile: " << filename << endl;
	printf(" already to save \n");
	return 0;
}
int video_init()
{
	capture.open("yang.mp4");       //打开前景视频
	double rate = capture.get(CV_CAP_PROP_FPS);
	printf("the rate of origin is %lf \n",rate);

	nFrame=static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));  //获取总帧数
	int delay = cvRound(1000.0/rate);
	if(!capture.isOpened())
		return -1;

	captureb.open("cityriver_720p.mov");       //打开背景视频    cityriver_720p.mov     background.webm
	double rateb = capture.get(CV_CAP_PROP_FPS);
	printf("the rate of background is %lf \n",rateb);
	long nFrameb=static_cast<long>(captureb.get(CV_CAP_PROP_FRAME_COUNT));  //总帧数
	if(!captureb.isOpened())
		return -1;

}

