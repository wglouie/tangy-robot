/*
 * Geoff Louie & Frank Despond
 *
 * Version 2.0 ---> Refactored from Geoffs Code
 *
 * January 26, 2015
 */
#include "processedimage.h"

#define NODEBUG
Point2f largeGridSortCentre;


template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

bool largeGridSort(const Point2f& i, const Point2f& j)
{
    float iPolarAngle, jPolarAngle;
    /* Sort clockwise */
    iPolarAngle = atan2(i.y-largeGridSortCentre.y,i.x-largeGridSortCentre.x);
    jPolarAngle = atan2(j.y-largeGridSortCentre.y,j.x-largeGridSortCentre.x);

    return iPolarAngle < jPolarAngle;
}

void shGetRidOfColor(Mat inputMat, int color){
    //this loop goes over the image and changes any shade equal to color
    //to 0.
    for(int y=0;y<inputMat.size().height;y++)
    {
        uchar *row = inputMat.ptr(y);
        for(int x=0;x<inputMat.size().width;x++)
        {
            if(row[x]==color)
            {
                int area = floodFill(inputMat, Point(x,y), CV_RGB(0,0,0));
            }
        }
    }
}
Point shLargestFlood(Mat inputMat){
    //printf("inside largestFlood");
    int count=0;
    int max=-1;
    Point maxPt;

    for(int y=0;y<inputMat.size().height;y++)
    {
        uchar *row = inputMat.ptr(y);
        for(int x=0;x<inputMat.size().width;x++)
        {
            if(row[x]>=128)
            {
                int area = floodFill(inputMat, Point(x,y), CV_RGB(0,0,64));
                //printf("filling %d, %d gray\n", x, y);
                if(area>max)
                {
                    maxPt = Point(x,y);
                    max = area;
                }
            }
        }
    }
    return maxPt;
}

void ProcessedImage::undistortCard(Mat grid, Mat originalImage, Mat &undistortedGrid, vector< vector<Point> > contours, int largestSquare,bool cardOnly)
{
    int topLeft=0;
    int topRight=0;
    int bottomLeft=0;
    int bottomRight=0;

    largeGridSortCentre=closestBigGridCentroid;

    std::sort(contours[largestSquare].begin(),contours[largestSquare].end(),largeGridSort);
    topLeft = 0;
    topRight = 1;
    bottomLeft = 3;
    bottomRight = 2;

    Point ptTopLeft = contours[largestSquare][topLeft];
    Point ptTopRight = contours[largestSquare][topRight];
    Point ptBottomLeft = contours[largestSquare][bottomLeft];
    Point ptBottomRight = contours[largestSquare][bottomRight];

    int maxLength = (ptBottomLeft.x-ptBottomRight.x)*(ptBottomLeft.x-ptBottomRight.x) + (ptBottomLeft.y-ptBottomRight.y)*(ptBottomLeft.y-ptBottomRight.y);
    int temp = (ptTopRight.x-ptBottomRight.x)*(ptTopRight.x-ptBottomRight.x) + (ptTopRight.y-ptBottomRight.y)*(ptTopRight.y-ptBottomRight.y);
    if(temp>maxLength) maxLength = temp;

    temp = (ptTopRight.x-ptTopLeft.x)*(ptTopRight.x-ptTopLeft.x) + (ptTopRight.y-ptTopLeft.y)*(ptTopRight.y-ptTopLeft.y);
    if(temp>maxLength) maxLength = temp;

    temp = (ptBottomLeft.x-ptTopLeft.x)*(ptBottomLeft.x-ptTopLeft.x) + (ptBottomLeft.y-ptTopLeft.y)*(ptBottomLeft.y-ptTopLeft.y);
    if(temp>maxLength) maxLength = temp;

    maxLength = sqrt((double)maxLength);

    if(cardOnly)
    {
        Point2f src[4], dst[4];
        src[0] = ptTopLeft;            dst[0] = Point2f(0,0);
        src[1] = ptTopRight;	       dst[1] = Point2f(maxLength-1, 0);
        src[2] = ptBottomRight;        dst[2] = Point2f(maxLength-1, maxLength-1);
        src[3] = ptBottomLeft;         dst[3] = Point2f(0, maxLength-1);

        //        line( originalImage, src[0] , src[1], Scalar(0, 255, 0), 6 );			//comment out
        //        line( originalImage, src[1] , src[2], Scalar(0, 255, 0), 6 );			//comment out
        //        line( originalImage, src[2] , src[3], Scalar(0, 255, 0), 6 );			//comment out
        //        line( originalImage, src[3] , src[0], Scalar(0, 255, 0), 6 );			//comment out





        undistortedGrid = Mat(Size(maxLength, maxLength), CV_8UC1);
        cv::warpPerspective(grid, undistortedGrid, cv::getPerspectiveTransform(src, dst), Size(maxLength, maxLength));
    }
    else
    {

        Point2f undistortSrc[4], undistortDst[4];
        undistortSrc[0] = ptTopLeft;            undistortDst[0] = Point2f(600,600);
        undistortSrc[1] = ptTopRight;	        undistortDst[1] = Point2f(maxLength+600, 600);
        undistortSrc[2] = ptBottomRight;        undistortDst[2] = Point2f(maxLength+600, maxLength+600);
        undistortSrc[3] = ptBottomLeft;         undistortDst[3] = Point2f(600, maxLength+600);
        undistortedGrid = Mat(Size(1920, 1080), CV_8UC1);;

        cv::warpPerspective(originalImage, undistortedGrid, cv::getPerspectiveTransform(undistortSrc, undistortDst), Size(1600, 1400));
        //imshow("Undistorted original Image",undistortedGrid);
        //waitKey();
    }
}


/*
 * Class Methods
 *
 * Processes image to find all the cards, their respective grids and centroids
 *
 * Returns only the largest (Closest) card
 */

ProcessedImage::ProcessedImage(Mat rawImage):
    cardFinder(), //initialize SquareFinder Class
    debugger(debug),
    bigGridCentroid(0.0,0.0),
    counter(0),
    largestSquare(-1),
    closestCardLargestSquare(-1),
    closestBigGridCentroid(0,0),
    distortedContours(0),
    squareCheck(0),
    gridCorners(0),
    cardCorners(0),
    centroids(0),
    closestCardDistortedContours(0),
    closestCardCentroids(0),
    counted(0),
    multipleCards(0)



{
    rawImage.copyTo(image);
    image.copyTo(finalImage);
    image.copyTo(testImage);

    image.copyTo(protectedImage);

    bingoCardCentroids();
}

ProcessedImage::~ProcessedImage()
{
    //printf("\n\nFIN----->ProcessedImage");

}

Mat ProcessedImage::getImage()
{
    return closestCardFinalImage; //should add guards
}

Mat ProcessedImage::getBigGrid()
{
    return closestCardBigGrid;
}

vector<Point2f> ProcessedImage::getCentroids()
{
    return closestCardCentroids;
}

Point2f ProcessedImage::getCardCentroidLocation()
{
    return bigGridCentroid;
}

Mat ProcessedImage::getUndistortedImage()
{
    return undistortedImage;
}



void ProcessedImage::bingoCardCentroids() //TODO: TEAR DOWN TO SMALLER CLASSES....

{
    while(findMultipleCards(image));

    //printf("\n------>Finished Searching<---------\n");
    //printf("\n------>Number of Cards Found: %d<---------\n", multipleCards.size());

    if(multipleCards.size()>0)
    {
        selectLargestGrid();
        //printf("\n------>Finished Finding Largest Grid<---------\n");



        if(closestCardDistortedContours.empty())
        {
            //printf("error with images (Blank Vector)\n");
            closestCardCentroids.empty();
        }
        else
        {
            Mat cardColorExtract = Mat(closestCardBigGrid.size(),CV_8UC3);
            protectedImage.copyTo(cardColorExtract,closestCardBigGrid);


            Mat cardColorUndistorted;
            Mat undistortedGrid;

            undistortCard(closestCardCdst,closestCardFinalImage,undistortedImage,closestCardDistortedContours,closestCardLargestSquare,false);		//Undistort original image for symbol
            undistortCard(closestCardCdst,closestCardFinalImage,undistortedGrid,closestCardDistortedContours,closestCardLargestSquare,true);			//Undistort color grid image
            undistortCard(cardColorExtract,closestCardFinalImage,cardColorUndistorted,closestCardDistortedContours,closestCardLargestSquare,true);//Undistort grid image extracted


            //FIND SQUARE CENTROIDS
            vector< vector<Point> >contours;    					//Vector of contours which consists of points

            cardFinder.findSquaresInGrid(undistortedGrid,contours);					//Find squares
            Mat blackGrid;
            undistortedGrid.copyTo(blackGrid);

            ///// Get the moments for each box contour
            vector<Moments> mu(contours.size() );

            for( int i = 0; i < contours.size(); i++ )
            {
                mu[i] = moments( Mat(contours[i]), false );
            }

            /////  Get the centroids(mass centers) of each contour:
            vector<Point2f> mc( contours.size() );

            // printf("\n------> Passed <---------\n");

            for( int i = 0; i < contours.size(); i++ )
            {
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );							//input contour centroid points
                circle(blackGrid,Point((int)mc[i].x,(int)mc[i].y),1,CV_RGB(0,255,0),5,8,0);					//Draw them nice and good
            }

            //printf("\nThe detected size of the boxes is %i\n",contours.size());
            closestCardCentroids = mc;
            closestCardBigGrid = cardColorUndistorted;

            #ifdef DEBUG
            time_t rawtime;
            struct tm * timeinfo;

            time (&rawtime);
            timeinfo = localtime (&rawtime);

            std::string imageResult;       		   // string which will contain the result
            std::ostringstream imageConvert;  			   // stream used for the conversion
            imageConvert << asctime(timeinfo);     // insert the textual representation of 'Number' in the characters in the stream
            imageResult = imageConvert.str(); // set 'Result' to the contents of the stream

           // cv::imwrite(("..//bingoTestResults//cardCentroids" + imageResult + ".jpg").c_str(),blackGrid);
            cv::imwrite(("..//Results//undistortedGrid" + imageResult + ".jpg").c_str(),closestCardBigGrid);
            #endif

        }

    }
    else
    {
        //printf("\n\n----->Failed----Processing Image\n\n");

        closestCardDistortedContours.empty();

    }
    //printf("\n\n----->Exit----Processing Image\n\n");
}

int ProcessedImage::findBingoGrid(Mat input, int process)
{
    Mat outerBox, element, inputImage;

    input.copyTo(inputImage);

    // printf("\n\n**************FINDING IMAGE********\n");

    if(process == 0)
        erode( inputImage, inputImage, element );
    else if(process == 1)
    {
        erode( inputImage, inputImage, element );
        erode( inputImage, inputImage, element );
    }
    else if(process == 2)
    {
        whiteBalance(inputImage);
    }
    else if(process == 3)
    {
        whiteBalance(inputImage);
        erode( inputImage, inputImage, element );
    }
    else if(process == 4)
    {
        whiteBalance(inputImage);
        erode( inputImage, inputImage, element );
        erode( inputImage, inputImage, element );
    }
    else
    {
        //do nothing
    }

    cvtColor(inputImage,inputImage,CV_RGB2GRAY);
    cv::GaussianBlur(inputImage,inputImage,Size(3,3),0);						//Remove noise for further processing

    adaptiveThreshold(inputImage, outerBox, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 9);	//Adaptive threshold acts like edge detection
    bitwise_not(outerBox, outerBox);							//Reverses pixel colours
    Point maxPt = shLargestFlood(outerBox);						//Find largest convex area(blob) in image because most likely bingo card

    floodFill(outerBox, maxPt, CV_RGB(255,255,255));	//color inside of bingo card area white
    shGetRidOfColor(outerBox, 64);								//Removing everything that isn't the largest blob
    vector<Vec4i> lines;                                        //Hough lines for the grid
    HoughLinesP(outerBox, lines, 1, CV_PI/180, 100, 10, 100);	//Find all the  lines in the grid and try to connect some also (if there is a problem finding grid lines change some values)

    cdst = Mat::zeros(outerBox.size(), CV_8UC3);
    Mat temp;

    input.copyTo(temp);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
        line( temp, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
#ifdef DEBUG
    std::string imageResult;       		   // string which will contain the result
    std::ostringstream imageConvert;  			   // stream used for the conversion
    imageConvert << counted;     // insert the textual representation of 'Number' in the characters in the stream
    imageResult = imageConvert.str();
    cv::imwrite(("..//paperPictures//foundGrid"+ imageResult+ ".jpg").c_str(),temp);
#endif
    counted++;

    //Undistort images using four corners of the bingo card
    vector<Vec4i>hierarchy;
    CvSize sz = Size( cdst.size().width, cdst.size().height);
    Mat timg = cdst.clone(); // make a copy of input image
    Mat gray = Mat(sz, CV_8UC1);
    Mat tgray;
    tgray = Mat(sz, CV_8UC1);
    cvtColor(timg,tgray,CV_RGB2GRAY);
    threshold(tgray,tgray,10,255,THRESH_BINARY);
    threshold( tgray, gray, 50, 255, CV_THRESH_BINARY );

    findContours( gray,distortedContours,hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

    //TODO: SQUARE FINDER
    cardFinder.findSquaresInGrid(timg,squareCheck);					//Find squares
    //printf("Number of squares: %i\n",squareCheck.size());

    if(squareCheck.size() != 26)
    {
        //printf("\n--->NO GRID FOUND<-------\n");
        centroids.empty();
        return 0;
    }
    else
    {
        largestSquare = cardFinder.findOuterBoxOfGrid(distortedContours);
        bigGrid = Mat::zeros(protectedImage.size(),CV_8UC1);	    //save a copy of just the big grid for later
        bigGrid = cardFinder.drawSquares(bigGrid,distortedContours,largestSquare,0);

        return 1;

    }
}
void ProcessedImage::reduceImageSize(Mat &imageToReduce, Mat &regionOfInterest,bool inverse, bool white)
{
    int color = 0;
    if(white)
    {
        color = 255;
    }
    //check to make sure images are the same size...
    if(imageToReduce.cols == regionOfInterest.cols &&
            imageToReduce.rows == regionOfInterest.rows)
    {
        for (int i = 0; i < imageToReduce.cols; i++)
        {
            for (int j = 0; j < imageToReduce.rows; j++)
            {
                Vec3b &intensityTarget = imageToReduce.at<Vec3b>(j, i);
                Vec3b &intensityROI = regionOfInterest.at<Vec3b>(j, i);

                if(inverse)
                {
                    if (  intensityROI.val[0] == 255 && intensityROI.val[1] == 255 && intensityROI.val[2] == 255)
                    {
                        intensityTarget.val[0] = color;
                        intensityTarget.val[1] = color;
                        intensityTarget.val[2] = color;
                    }

                    else
                    {

                    }

                }
                else{

                    if (  intensityROI.val[0] == 255 && intensityROI.val[1] == 255 && intensityROI.val[2] == 255)
                    {

                    }

                    else
                    {
                        intensityTarget.val[0] = color;
                        intensityTarget.val[1] = color;
                        intensityTarget.val[2] = color;
                    }
                }
            }
        }

    }
}
//http://pippin.gimp.org/image_processing/chapter-automaticadjustments.html#section-contraststretching

//White balance adjustment
void ProcessedImage::whiteBalance(Mat &image)
{
    float total [4]; //Total intensity
    float min [4]; // {blueMax, greenMax, redMax}
    float max [4]; // {blueMax, greenMax, redMax}

    for (int i = 0; i < image.cols; i++)
    {
        for (int j = 0; j < image.rows; j++)
        {
            Vec3b &intensity = image.at<Vec3b>(j, i);

            for(int k = 0; k < image.channels(); k++)
            {
                total[k] += intensity.val[k]; //b, g, r intensities
            }
        }
    }
    float blue, green, red, alpha;

    blue = total[0]/(image.cols*image.rows);
    green = total[1]/(image.cols*image.rows);
    red = total[2]/(image.cols*image.rows);
    alpha = total[3]/(image.cols*image.rows);

    float average [] = {blue, green, red, alpha};
    min[0]=max[0]= blue;
    min[1]=max[1]=green;
    min[2]=max[2]=red;
    min[3]=max[3]=alpha;

    for (int i = 0; i < image.cols; i++)
    {
        for (int j = 0; j < image.rows; j++)
        {
            Vec3b &intensity = image.at<Vec3b>(j, i);

            for(int k = 0; k < image.channels(); k++)
            {
                if(min[k]>intensity.val[k])
                {
                    //If min is bigger than the value, set min to the value
                    min[k]=intensity.val[k];
                }
                if(max[k]<intensity.val[k])
                {
                    //If min is bigger than the value, set min to the value
                    max[k]=intensity.val[k];
                }
            }
        }
    }
    float adjustmentblue, adjustmentgreen, adjustmentred;
    adjustmentblue = max[1]/max[0];
    adjustmentgreen = 1;
    adjustmentred = max[1]/max[2];

    for (int i = 0; i < image.cols; i++)
    {
        for (int j = 0; j < image.rows; j++)
        {
            Vec3b &intensity = image.at<Vec3b>(j, i);

            for(int k = 0; k < image.channels(); k++)
            {
                if(k==0)
                {

                    intensity.val[k] = (intensity.val[k]*adjustmentblue);
                }
                else if (k==1)
                {

                    intensity.val[k] = (intensity.val[k]*adjustmentgreen);
                }
                else if (k==2)
                {


                    intensity.val[k] = (intensity.val[k]*adjustmentred);
                }
                else
                {
                    intensity.val[k] = ((intensity.val[k]-min[k])*(255/(max[k]-min[k])));
                }
            }
        }
    }
}

//recursive function finding all the cards possible in the one image
int ProcessedImage::findMultipleCards(Mat inputImage)
{
    Mat imageNoGrid;
    bool foundCard = false;
    for(int i =0; i<numberOfProcesses; i++)
    {
        if (findBingoGrid(inputImage, i))
        {
            counter++;
            //printf("\n----->COUNTER :%d<--------\n", counter);
            //debugger.imageToScreen("inputImage", inputImage, __LINE__, __FILE__);
            //debugger.imageToScreen("inputImage", inputImage, __LINE__, __FILE__);

            Mat card = Mat::zeros(inputImage.size(),CV_8UC3);
            Mat card2 = Mat::zeros(inputImage.size(),CV_8UC3);

            float sizeEdgeA, sizeEdgeB, sizeEdgeC, sizeEdgeD, averageLength;



            // find the centre of the grid by adding up x coordinates and divided by 4; same as y coordinates
            bigGridCentroid.x=(fabs(distortedContours[largestSquare][0].x + distortedContours[largestSquare][1].x +
                    distortedContours[largestSquare][2].x + distortedContours[largestSquare][3].x)/4);
            bigGridCentroid.y=fabs(distortedContours[largestSquare][0].y + distortedContours[largestSquare][1].y+
                    distortedContours[largestSquare][2].y + distortedContours[largestSquare][3].y)/4;



            //Used in the sorting algorithm
            largeGridSortCentre = bigGridCentroid;

            //Organizes points clockwise
            std::sort(distortedContours[largestSquare].begin(),distortedContours[largestSquare].end(),largeGridSort);

            /*
   * Once the centre of the grid is found, it is copied and expanded
   * This allows for the image to be reduced in size to focus on just the card of interest
   */


            gridCorners.push_back(distortedContours[largestSquare][0]);
            gridCorners.push_back(distortedContours[largestSquare][1]);
            gridCorners.push_back(distortedContours[largestSquare][2]);
            gridCorners.push_back(distortedContours[largestSquare][3]);
            //std::sort(gridCorners.begin(),gridCorners.end(),largeGridSort);

            //Finds the lengths of the grids to find the average lenght
            sizeEdgeA = sqrt((pow((distortedContours[largestSquare][0].x -distortedContours[largestSquare][1].x), 2) + pow((distortedContours[largestSquare][0].y -distortedContours[largestSquare][1].y),2) ));
            sizeEdgeB = sqrt((pow((distortedContours[largestSquare][1].x -distortedContours[largestSquare][2].x),2) + pow((distortedContours[largestSquare][1].y -distortedContours[largestSquare][2].y),2)));
            sizeEdgeC = sqrt((pow((distortedContours[largestSquare][2].x -distortedContours[largestSquare][3].x),2) + pow((distortedContours[largestSquare][2].y -distortedContours[largestSquare][3].y),2)));
            sizeEdgeD = sqrt((pow((distortedContours[largestSquare][3].x -distortedContours[largestSquare][0].x),2) + pow((distortedContours[largestSquare][3].y -distortedContours[largestSquare][0].y),2)));

            averageLength = (sizeEdgeA+sizeEdgeB+sizeEdgeC+sizeEdgeD)/4;
            // printf("\naverageLength = %f\n", averageLength);
            averageLength = averageLength*0.75;

            //average length is added or subtracted to the
            //TopLeft bigger
            cardCorners.push_back(Point(gridCorners[0].x-averageLength, gridCorners[0].y-averageLength));
            //Topright Bigger
            cardCorners.push_back(Point(gridCorners[1].x+averageLength, gridCorners[1].y-averageLength));
            //Bottom Left Bigger
            cardCorners.push_back(Point(gridCorners[2].x+averageLength, gridCorners[2].y+averageLength));
            //bottom right bigger
            cardCorners.push_back(Point(gridCorners[3].x-averageLength, gridCorners[3].y+averageLength));

            //check new grid corners are within bounds

            for(int i = 0; i<4; i++)
            {
                if(cardCorners[i].x<0) cardCorners[i].x=0;
                if(cardCorners[i].y<0) cardCorners[i].y=0;
                if(cardCorners[i].x>protectedImage.cols) cardCorners[i].x=protectedImage.cols;
                if(cardCorners[i].y>protectedImage.rows) cardCorners[i].y=protectedImage.rows;
            }
            Mat outImage, outBigGrid, outCdst;

            bigGrid.copyTo(outBigGrid);
            cdst.copyTo(outCdst);
            finalImage.copyTo(outImage);
            vector< vector<Point> > outDistortedContours = distortedContours;
            //float sizeOfCardGrid = averageLength;

            int outLargestSquare = largestSquare;
            Point2f outBigGridCentroid = bigGridCentroid;


            floodFill(outBigGrid,outBigGridCentroid,CV_RGB(255,255,255));

            line(card2, gridCorners[0], gridCorners[1], Scalar(255,255,255), 1, 8, 0);
            line(card2, gridCorners[1], gridCorners[2], Scalar(255,255,255), 1, 8, 0);
            line(card2, gridCorners[2], gridCorners[3], Scalar(255,255,255), 1, 8, 0);
            line(card2, gridCorners[3], gridCorners[0], Scalar(255,255,255), 1, 8, 0);


            inputImage.copyTo(imageNoGrid);
            floodFill(card2,outBigGridCentroid,CV_RGB(255,255,255));

            reduceImageSize(imageNoGrid, card2, true,true);

            line(card, cardCorners[0], cardCorners[1], Scalar(255,255,255), 1, 8, 0);
            line(card,cardCorners[1], cardCorners[2], Scalar(255,255,255), 1, 8, 0);
            line(card, cardCorners[2], cardCorners[3], Scalar(255,255,255), 1, 8, 0);
            line(card, cardCorners[3], cardCorners[0], Scalar(255,255,255), 1, 8, 0);

            floodFill(card,outBigGridCentroid,CV_RGB(255,255,255));
            float sizeOfCardGrid = sizeOfGrid(card2);

            reduceImageSize(outImage, card, false, false);

            multipleCards.push_back({outImage, outBigGrid, outCdst, outDistortedContours, outLargestSquare, outBigGridCentroid, sizeOfCardGrid});
            foundCard = true;

            imageNoGrid.copyTo(image);

            break;
        }

    }

    if(foundCard)
    {
        // printf("\n------>Found A Card<---------\n");
        reset();

        return 1;
    }
    else
    {
        //printf("\n\n------>Finished Multicard Search<-------");
        return 0;
    }
}



float ProcessedImage::sizeOfGrid(Mat &imageOfGrid)
{
    //Return size of grid area;

    float size = 0.0;

    for (int i = 0; i < imageOfGrid.cols; i++)
    {
        for (int j = 0; j < imageOfGrid.rows; j++)
        {
            Vec3b &intensityGrid = imageOfGrid.at<Vec3b>(j, i);

            if (  intensityGrid.val[0] == 255 && intensityGrid.val[1] == 255 && intensityGrid.val[2] == 255)
            {

                size++;
            }


        }
    }

    return size;


}

void ProcessedImage::selectLargestGrid()
{
    //check for multiple cards
    //if More than 1 found, determine the biggest and set them as the closestCard
    int largestGrid =0;

    if(multipleCards.size()>1)
    {
        for(int i = 1; i <multipleCards.size();i++)
        {
            if(multipleCards[i].sizeOfBingoGrid > multipleCards[largestGrid].sizeOfBingoGrid) largestGrid = i;
            // printf("\nBIGGEST CARD :%i", largestGrid);

        }

        closestBigGridCentroid =multipleCards[largestGrid].individualBigGridCentroid;

        closestCardFinalImage =multipleCards[largestGrid].individualBingoCard;

        closestCardCdst =multipleCards[largestGrid].individualBingoCdst;

        closestCardBigGrid =multipleCards[largestGrid].individualBigGrid;

        closestCardDistortedContours =multipleCards[largestGrid].individualDistoredContours;

        closestCardLargestSquare =multipleCards[largestGrid].largestIndividualSquare;

    }
    else
    {
        //Return the one card;
        closestBigGridCentroid =multipleCards[largestGrid].individualBigGridCentroid;

        closestCardFinalImage =multipleCards[largestGrid].individualBingoCard;

        closestCardCdst =multipleCards[largestGrid].individualBingoCdst;

        closestCardBigGrid =multipleCards[largestGrid].individualBigGrid;

        closestCardDistortedContours =multipleCards[largestGrid].individualDistoredContours;

        closestCardLargestSquare =multipleCards[largestGrid].largestIndividualSquare;

    }


}


void ProcessedImage::reset()
{

    protectedImage.copyTo(finalImage);
    squareCheck.clear();
    distortedContours.clear();
    bigGrid =Scalar(0,0,0);
    largestSquare = 0;


    gridCorners.clear();
    cardCorners.clear();

    bigGridCentroid = Point(0,0);

    centroids.clear();

}



