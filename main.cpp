#include "sharedmatting.h"
#include <string>

using namespace std;
using namespace cv;

/****��������****/
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


/****ȫ�ֱ���****/
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
	/*****����λ���Լ���С*****/
	while(capture.read(frame) && !flag_began  && captureb.read(background))
	{
		Mat test =  Test_composition(frame,background);
		imshow("the composition picture",test);
		waitKey(-1);
	}
	cvDestroyWindow("the composition picture");
	printf("now beganing to produce compostion video.......\n");

	///*****����װ����Ƶ��ǰ�������ǰ֡���ܲ�����ʼ֡������ʼ�ϳ�*****/
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

		////��ʼ��ȡalpha
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
		resize(frame,frame,frame.size()/2);      //ԭͼ����ֻ��������С��
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
		
		////��ʼ��ȡalpha
		SharedMatting sm;
		sm.loadImage("input/input.png");
		sm.loadTrimap("trimap/trimap.png");
		sm.solveAlpha();
		sprintf(fileAddr, "result/result.png");
		sm.save(fileAddr);

		//��ʼ�ϳ�
		Mat alpha = imread("result/result.png",0);
		Mat out = compostion(alpha,part,background);	
		//imshow("the alpha is ",alpha);
		return out;
}
Mat Test_composition(Mat frame,Mat background)
{
		resize(frame,frame,frame.size()/2);      //ԭͼ����ֻ��������С��
		Mat part=frame(rect);
		resize(part,part,Size(0,0),proportion,proportion,INTER_LINEAR);
		rows_p=part.rows; 
		cols_p=part.cols;
		Mat mask = getmask1(part);
		compostion_first(part,background,mask);
		return background;
}

//���ڻ�ȡtrimap
Mat getmask(Mat part)
{
	Mat img,mask;
	cvtColor(part, img, COLOR_BGR2HSV);      //��frame��RGBת��ΪHSV

	inRange(img, Scalar(35, 103, 46), Scalar(155, 255, 255), mask); // ��ֵ�����ֱ����ߵ���ֵ

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));  //����ṹԪ��
	morphologyEx(mask, mask, MORPH_CLOSE, element);					// ������    ȥ����ɫǰ��ɫ�ϵİ׵㣨ȥ���ģ�  ��ͬ������ͬ����
	//erode(mask, mask, element);										// ��ʴ ��Բ���߽磬Ĭ��ִ��һ�Σ��ɸ�
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

//����Ȧ��Ŀ��
Mat getmask1(Mat part)
{
	Mat img,mask;
	cvtColor(part, img, COLOR_BGR2HSV);      //��frame��RGBת��ΪHSV

	inRange(img, Scalar(35, 103, 46), Scalar(155, 255, 255), mask); // ��ֵ�����ֱ����ߵ���ֵ

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));  //����ṹԪ��
	morphologyEx(mask, mask, MORPH_CLOSE, element);					// ������    ȥ����ɫǰ��ɫ�ϵİ׵㣨ȥ���ģ�  ��ͬ������ͬ����
	//erode(mask, mask, element);										// ��ʴ ��Բ���߽磬ȥ���׵㣬Ĭ��ִ��һ�Σ��ɸ�
	
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

vector<vector<Point>> contours;                    //��ѭ���ڲ�����
vector<Vec4i> hierarchy;
CvRect get_point(Mat out)                           //�õ����б�Ե��
{
	//�ο�   https://blog.csdn.net/dcrmg/article/details/51987348
	CvRect point;
	findContours(out,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);		//modeȡֵ��CV_RETR_EXTERNAL����methodȡֵ��CV_CHAIN_APPROX_NONE������ֻ�����������������ұ������������е�
																							// modeȡֵ��CV_RETR_LIST����methodȡֵ��CV_CHAIN_APPROX_SIMPLE���������������������������֮��˴˶������������ȼ���ϵ�����ҽ����������Ϲյ���Ϣ��

	Mat imageContours=Mat::zeros(out.size(),CV_8UC1);
	Mat Contours=Mat::zeros(out.size(),CV_8UC1); 
	//Mat rect = Mat::zeros(out.size(),CV_8UC3);
	Mat rect=out;

	for(int i=0;i<contours.size();i++)
	{
		CvRect boundRect=boundingRect(contours[i]);

		//�˴�ע��Ϊ������������

		////contours[i]������ǵ�i��������contours[i].size()������ǵ�i�����������е����ص���		
		//for(int j=0;j<contours[i].size();j++) 	
		//{		
		//			//���Ƴ�contours���������е����ص�		
		//	Point P=Point(contours[i][j].x,contours[i][j].y);		
		//	Contours.at<uchar>(P)=255;	
		//} 	
		//		//���hierarchy��������		
		//		char ch[256];		
		//		sprintf(ch,"%d",i);		
		//		string str=ch;	
		//		cout<<"����hierarchy�ĵ�" <<str<<" ��Ԫ������Ϊ��"<<endl<<hierarchy[i]<<endl<<endl; 	
		////��������		
		//drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);

		if(boundRect.width>100 && boundRect.height>100 && boundRect.width <1000)    //ֻ����Ŀ����������Ŀ�
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
	//imshow("Contours Image",imageContours); //����
	//imshow("Point of Contours",Contours);   //����contours�ڱ�������������㼯
	printf("\n the rect: x=%d,y=%d,h=%d,W=%d \n",point.x,point.y,point.height,point.width);
	return point;
}

void on_mouse(int event,int x,int y, int flags,void* ustc)    //������ʲô��˼
{
	static CvPoint pre_pt =(-1,-1);
	static CvPoint cur_pt =(-1,-1);
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);

	if(event == CV_EVENT_LBUTTONDOWN)							//��������¼�
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
	//else if(event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))   //�ƶ����
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
	////ע��  ֻ�������бպ�������ͼ��
	rect=get_point(mask1);

	//���û�������������¼��Ե�����С��λ��
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
	capture.open("yang.mp4");       //��ǰ����Ƶ
	double rate = capture.get(CV_CAP_PROP_FPS);
	printf("the rate of origin is %lf \n",rate);

	nFrame=static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));  //��ȡ��֡��
	int delay = cvRound(1000.0/rate);
	if(!capture.isOpened())
		return -1;

	captureb.open("cityriver_720p.mov");       //�򿪱�����Ƶ    cityriver_720p.mov     background.webm
	double rateb = capture.get(CV_CAP_PROP_FPS);
	printf("the rate of background is %lf \n",rateb);
	long nFrameb=static_cast<long>(captureb.get(CV_CAP_PROP_FRAME_COUNT));  //��֡��
	if(!captureb.isOpened())
		return -1;

}

