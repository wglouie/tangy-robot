/*
 * Frank Despond
 *
 * Version 1.0
 *
 * January 28, 2015
 *
 * ///TODO: change it so its more contained --->less pass by reference.
 *
 */
#include "squarefinder.h"

SquareFinder::SquareFinder()
{
    numberOfSquaresInGrid=0;
}

SquareFinder::~SquareFinder()
{
}
double angle(Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
//finds biggest square
void SquareFinder::findBiggestSquare(Mat inputImage, vector<vector<Point> > &bigSquare)
{
    Mat blurred, gray;


    vector<Mat> channels;
    Mat img_hist_equalized;

    float tot [3]; // {blueAverage, greenAverage, redAverage}

    for (int i = 0; i < inputImage.cols; i++)
    {
        for (int j = 0; j < inputImage.rows; j++)
        {
            Vec3b intensi = inputImage.at<Vec3b>(j, i);

            for(int k = 0; k < inputImage.channels(); k++)
            {
                tot[k] += intensi.val[k]; //b, g, r intensities
            }
        }
    }
    // Calculate average pixel intensity.....

    float blu, gree, re;

    blu = tot[0]/(inputImage.cols*inputImage.rows);
    gree = tot[1]/(inputImage.cols*inputImage.rows);
    re = tot[2]/(inputImage.cols*inputImage.rows);



    //get the average inensity for each color

    //printf("blue average %f\n", blu);
    //printf("green average %f\n", gree);
    //printf("red average %f\n", re);

    int thresh;

    if (blu > 120 && re >120 && gree >120)
    {
        thresh = 110;
    }

    if (blu <95 && re< 95 && gree <95)
    {
        thresh = 90;
    }
    else
    {
        thresh = 100;
    }
    //Added contrast boost to better detect edges

    cvtColor(inputImage, img_hist_equalized, CV_BGR2YCrCb);

    split(img_hist_equalized, channels);

    equalizeHist(channels[0], channels[0]);

    merge(channels, img_hist_equalized);

    cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR);
    img_hist_equalized.convertTo(img_hist_equalized, -1, 0.7, 0);

    cvtColor(img_hist_equalized, img_hist_equalized, CV_8UC1);

    // blur will enhance edge detection
    cvtColor(img_hist_equalized,img_hist_equalized,CV_RGB2GRAY);
    // blur( img_hist_equalized, img_hist_equalized, Size(2,2) );
    medianBlur(img_hist_equalized, img_hist_equalized, 13);

    threshold( img_hist_equalized, blurred, thresh, 255, THRESH_TOZERO );
    //fitImageToScreen("Thresholding Output", blurred);
    //waitKey();
    //threshold( blurred, blurred, 230, 255, THRESH_BINARY );
    // Generate grad_x and grad_y
    // find the gradient

    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    Mat grad_x, grad_y, grad;
    Mat abs_grad_x, abs_grad_y;

    //Also calculated gradients based on threshold of the image above for better paper detection

    // Gradient X
    //Scharr(blurred, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( blurred, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    // Gradient Y
    //Scharr(blurred, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( blurred, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    // Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );


    //imshow("Threshold2", grad);
    //imshow("Threshold", blurred);
    //fitImageToScreen("Gradient", grad);
    //waitKey();
    //image.copyTo(blurred);
    //medianBlur(blurred, blurred, 13);

    //Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    //for (int c = 0; c < 3; c++)
    //{
    //int ch[] = {c, 0};
    //mixChannels(&blurred, 1, &gray0, 1, ch, 1);

    // try several threshold levels
    const int threshold_level = 2;
    for (int l = 0; l < threshold_level; l++)
    {
        // Use Canny instead of zero threshold level!
        // Canny helps to catch squares with gradient shading
        if (l == 0)
        {
            int can = 30;
            int threecan = can*3;
            Canny(grad, gray, can, threecan, 3); //

            // Dilate helps to remove potential holes between edge segments
            dilate(gray, gray, Mat(), Point(-1,-1));
        }
        else
        {
            gray = grad >= (l+1) * 255 / threshold_level;
        }

        // Find contours and store them in a list
        findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        // Test contours
        vector<Point> approx;
        int largestSquareArea = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

            // Note: absolute value of an area is used because
            // area may be positive or negative - in accordance with the
            // contour orientation
            if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)))
            {
                double maxCosine = 0;

                for (int j = 2; j < 5; j++)
                {
                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    maxCosine = MAX(maxCosine, cosine);
                }

                if (maxCosine < 0.80 && largestSquareArea < fabs(contourArea(Mat(approx))))
                {
                    if(bigSquare.size() != 0)
                        bigSquare.pop_back();
                    bigSquare.push_back(approx);
                    largestSquareArea = fabs(contourArea(Mat(approx)));
                }
            }
        }
    }

}
//finds squares in a grid pattern
void SquareFinder::findSquaresInGrid(Mat inputImage, vector<vector<Point> > &gridSquares)
{
    vector< vector<Point> >contours;
    vector< vector<Point> >squaresTemp;
    vector<Vec4i>hierarchy;
    int boxCount = 0;
    int N = 11;
    CvSize sz = Size( inputImage.size().width, inputImage.size().height);
    Mat timg = inputImage.clone(); // make a copy of input image
    Mat gray = Mat(sz, CV_8UC1);
    Mat tgray;

    double s, t;
    tgray = Mat(sz, CV_8UC1);
    cvtColor(timg,tgray,CV_RGB2GRAY);
    threshold(tgray,tgray,10,255,THRESH_BINARY);
    threshold( tgray, gray, 50, 255, CV_THRESH_BINARY );

    findContours( gray,contours,hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

    // test each contour
    for(int j = 0;j<contours.size();j++)
    {
        //printf("%f\n",arcLength(Mat(contours[j],0),0));
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[j]), contours[j],arcLength(Mat(contours[j],0),0)*0.03, 1);
        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation

        if( contours[j].size() == 4 && fabs(contourArea(Mat(contours[j]))) > 50) //&&
        {
            s = 0;
            for(int i = 0; i < 4; i++ )
            {
                // find minimum angle between joint
                // edges (maximum of cosine)
                if( i >= 2 )
                {
                    t = fabs(angle(contours[j][i], contours[j][i-2], contours[j][i-1]));
                    s = s > t ? s : t;
                }
            }
            // if cosines of all angles are small squares[1].x
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( s < 0.7 )
            {
                squaresTemp.push_back(vector<Point>());
                for(int i = 0; i < 4; i++ )
                {
                    squaresTemp[boxCount].push_back(contours[j][i]);
                }
                boxCount = boxCount + 1;
            }

        }
    }



    gridSquares = squaresTemp;
    numberOfSquaresInGrid = gridSquares.size();
}
//returns number of squares in a grid
int SquareFinder::getNumberOfSquaresInGrid()
{
    return numberOfSquaresInGrid;
}

//draws corresponding squares and returns a Mat
Mat SquareFinder::drawSquares(Mat img, vector<vector<Point> > squares, int squarePosition, bool multiSquare)
{
    if(squares.size() == 0)
    {
        return img;
    }

    Mat cpy = img.clone();

    // initialize reader of the sequence
    Point* rect = pt;
    // read 4 sequence elements at a time (all vertices of a square)
    if(multiSquare == 1)
    {
        for(int i = 0; i < squares.size(); i ++ )
        {

            rect[0]=squares[i][0];
            rect[1]=squares[i][1];
            rect[2]=squares[i][2];
            rect[3]=squares[i][3];
            int count = 4;
            //polylines( cpy,&rect,&count,1,1, CV_RGB(0,255,0), 3,8,0 );
            line(cpy,rect[0],rect[1], CV_RGB(0,255,0), 1,8,0);
            line(cpy,rect[1],rect[2], CV_RGB(0,255,0), 1,8,0);
            line(cpy,rect[2],rect[3], CV_RGB(0,255,0), 1,8,0);
            line(cpy,rect[3],rect[0], CV_RGB(0,255,0), 1,8,0);
            //imshow("DetectedBoxes", cpy );
            //imwrite("DetectedBoxes.jpg",cpy);
            //waitKey();

        }
    }
    else
    {
        rect[0]=squares[squarePosition][0];
        rect[1]=squares[squarePosition][1];
        rect[2]=squares[squarePosition][2];
        rect[3]=squares[squarePosition][3];
        int count = 4;
        //polylines( cpy,&rect,&count,1,1, CV_RGB(0,255,0), 3,8,0 );
        line(cpy,rect[0],rect[1], CV_RGB(255,255,255), 1,8,0);
        line(cpy,rect[1],rect[2], CV_RGB(255,255,255), 1,8,0);
        line(cpy,rect[2],rect[3], CV_RGB(255,255,255), 1,8,0);
        line(cpy,rect[3],rect[0], CV_RGB(255,255,255), 1,8,0);

    }
    return cpy;
    // show the resultant image

}

//finds largest square in the grid and returns it
int SquareFinder::findOuterBoxOfGrid(vector< vector<Point> > &squares)
{
    if (squares.size() == 0)
    {
        return 0;
    }
    float size = 0;
    float length = 0;
    int idx;

    for(int j = 0; j < squares.size(); j++)
    {
        approxPolyDP(Mat(squares[j]), squares[j],arcLength(Mat(squares[j],0),0)*0.03, 1);
        if( squares[j].size() == 4 && fabs(contourArea(Mat(squares[j]))) > 50) //&&
        {
            //for(int i = 0;i < squares.size();i++)
            //{
            length = arcLength(Mat(squares[j]),1);
            //printf("\nidx = %i Length = %f",j,length);
            if(size<length)
            {
                size = length;
                idx = j;
            }
            //}
        }
    }

    //printf("\nbiggestGrid = %i",idx);
    return idx;
}
