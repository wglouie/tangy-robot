/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetection Code
 *
 * January 29, 2015
 */
#include "redmarkers.h"

#define NODEBUG
//bool gridSortYValues(const Point2f& i, const Point2f& j)
//{
//    /* Sort from left to right, top to bottom */
//    if (i.y<j.y)
//        return true;
//    else
//        return false;
//}
//bool gridSortXValues(const Point2f& i, const Point2f& j)
//{
//    /* Sort from left to right, top to bottom */
//    if (i.x<j.x)
//        return true;
//    else
//        return false;
//}

Point2f RedMarkers::centroid(vector<Point> contour)
{
    ///// Get the moment for contour
    Moments mu;
    mu = moments( Mat(contour), false );

    /////  Get the centroid(mass centers) of contour:
    Point2f mc;														//Vector that holds centroid points
    mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );					//input contour centroid points
    return mc;
}

float ellipseFitMeasure(Mat blob,RotatedRect rect)
{
    Mat ellipseImg = Mat::zeros(blob.size(),CV_8UC3);
    float ellipseSize = 0;
    float pixelsInEllipse =	0;
    ellipse(ellipseImg,rect,CV_RGB(255,255,255),CV_FILLED,8);

    for(int i=0;i<ellipseImg.size().height;++i)
    {
        for(int j=0;j<ellipseImg.size().width;++j)
        {
            if(blob.at<Vec3b>(i,j) != Vec3b(0,0,0))
            {
                ellipseSize += 1;
                if(ellipseImg.at<Vec3b>(i,j)==blob.at<Vec3b>(i,j))
                    pixelsInEllipse += 1;

            }
        }
    }
    return pixelsInEllipse/ellipseSize;
}

int RedMarkers::closestSquare( Point2f circleOfInterest)
{

    for(int i = 0; i < individualSquares.size();i++)
    {
        int y =circleOfInterest.y;
        int x = circleOfInterest.x;
        Vec3b &intensity = individualSquares[i].at<Vec3b>(y, x);

        if(intensity.val[0] == 255 && intensity.val[1] == 255 && intensity.val[2] == 255)
        {
            // centroid inside square
            return i;
        }

    }
}

RedMarkers::RedMarkers(Mat inputCardGrid,
                       vector<int> inputCalledNumbers,
                       int inputGameType,
                       string inputNumberDatabaseFile,
                       int inputCardMarker,
                       vector<Point2f> inputCardGridCentroids):
    cardMarker(inputCardMarker), numberOfCards(0),
    calledNumbers(inputCalledNumbers), gameType(inputGameType),
    numberDatabaseFile(inputNumberDatabaseFile),
    debugger(debug),
    cardGridCentroids(inputCardGridCentroids),
    bingo(-1),
    checkBingoCard{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
    circleCentroids(0),
    markerCentroids(0),
    individualSquares(0),
    currentCardNumbers(0),
    markedNumbers(0),
    wrongNumbersMarked(0),
    missingNumbers(0),
    numberDatabase(0)
{
    //    std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),gridSortYValues);
    //    std::sort(cardGridCentroids.begin(),cardGridCentroids.end()-20,gridSortXValues);
    //    std::sort(cardGridCentroids.begin()+5,cardGridCentroids.end()-15,gridSortXValues);
    //    std::sort(cardGridCentroids.begin()+10,cardGridCentroids.end()-10,gridSortXValues);
    //    std::sort(cardGridCentroids.begin()+15,cardGridCentroids.end()-5,gridSortXValues);
    //    std::sort(cardGridCentroids.begin()+20,cardGridCentroids.end(),gridSortXValues);

    inputCardGrid.copyTo(cardGrid);

    grabNumberDatabase();

    currentCardNumbers = numberDatabase[cardMarker];

    circleFinder();
    bingo = checkNumbers();
    printOffResults();

}

RedMarkers::~RedMarkers()
{
    printf("\n\nFIN----->RED");


}

int RedMarkers::checkForBingo()
{
    return bingo;
}

vector<int> RedMarkers::getMissingNumbers()
{
    return missingNumbers;
}

vector<int> RedMarkers::getWrongNumbersMarked()
{
    return wrongNumbersMarked;
}

void RedMarkers::grabNumberDatabase()
{
    ifstream inFile(numberDatabaseFile.c_str());
    string line;
    while(getline(inFile, line)){
        istringstream iss(line);
        int temp;

        numberDatabase.push_back(vector<int>(25,0));

        //ROS_INFO("Card: %s", cardDatabase[numberOfCards].c_str());
        for(int i = 0; i < 25; i++)
        {
            if(iss >> temp)
                numberDatabase[numberOfCards][i] = temp;
            else
                break;
            //ROS_INFO("%i ",numberDatabase[numberOfCards][i]);
        }

        //printf("\n");
        numberOfCards++;
    }
    //printf("Number of Cards: %i\n", numberOfCards);
    numberOfCards = numberOfCards - 1;
    inFile.close();
}

bool RedMarkers::contains(int elem, vector<int> list)
{
    bool result = false;

    for(int i = 0; i < list.size(); i++){
        int current = list[i];
        if (current == elem){
            result = true;
            break;
        }
    }

    return result;

}

void RedMarkers::circleFinder()
{
    Mat img, grid, test;
    cardGrid.copyTo(test);
    cardGrid.copyTo(img);
    grid = Mat::zeros(img.size(), CV_8UC3);

    vector< vector<Point> >contours;
    vector<Vec4i>hierarchy;


    //Draw grid to seperate connecting circles
    for(int i=0; i<=img.rows; i+=img.rows/5)
    {
        cv::line(img,Point(0,i),Point(img.rows,i),cv::Scalar(0,0,0),5);
        cv::line(grid,Point(0,i),Point(grid.rows,i),cv::Scalar(255,0,0),5);

    }

    for(int i=0; i<=img.cols; i+=img.cols/5)
    {

        cv::line(img,Point(i,0),Point(i,img.cols),cv::Scalar(0,0,0),5);
        cv::line(grid,Point(i,0),Point(i,grid.cols),cv::Scalar(255,0,0),5);


    }
    Mat testCentroids, testGrid;

    grid.copyTo(testGrid);
    grid.copyTo(testCentroids);

    int averageGridSize = 0;

    for(int i = 0; i < cardGridCentroids.size();i++)
    {

        Mat littleSquare;
        grid.copyTo(littleSquare);

        floodFill(littleSquare, cardGridCentroids[i], CV_RGB(255,255,255));	//color inside of bingo card area white
        floodFill(testGrid, cardGridCentroids[i], CV_RGB(255,255,255));	//color inside of bingo card area white
        circle(testCentroids,cardGridCentroids[i],1,CV_RGB(0,255,0),5,8,0);					//Draw them nice and good

        for (int i = 0; i < littleSquare.cols; i++)
        {
            for (int j = 0; j < littleSquare.rows; j++)
            {
                Vec3b &intensity = littleSquare.at<Vec3b>(j, i);

                if(intensity.val[0] == 255 && intensity.val[1] == 255 && intensity.val[2] == 255)
                {
                    averageGridSize++;
                    //do nothing
                }
                else
                {
                    intensity.val[0] = 0;
                    intensity.val[1] = 0;
                    intensity.val[2] = 0;
                }

            }
        }


        //        imshow( "Square", testGrid);                   // Show our image inside it.
        //        imshow( "testCentroids", testCentroids);                   // Show our image inside it.

        //        waitKey(0);
        individualSquares.push_back(littleSquare);

    }
    averageGridSize = averageGridSize/25;

    printf("\n\n---->averageGridSize = %i", averageGridSize);

    for (int i = 0; i < img.cols; i++)
    {
        for (int j = 0; j < img.rows; j++)
        {
            Vec3b &inten = img.at<Vec3b>(j, i);
            float blue = inten.val[0];
            float green = inten.val[1];
            float red = inten.val[2];

            float redGreen = red/green;
            float redBlue = red/blue;


            if(redGreen > 1.2 && redBlue >1.2)
            {

            }
            else
            {
                inten.val[0] = 0;
                inten.val[1] = 0;
                inten.val[2] = 0;
            }

        }
    }

#ifdef DEBUG
    time_t rawtime;
    struct tm * timeinfo;

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    std::string imageResult;       		   // string which will contain the result
    std::ostringstream imageConvert;  			   // stream used for the conversion
    imageConvert << asctime(timeinfo);     // insert the textual representation of 'Number' in the characters in the stream
    imageResult = imageConvert.str(); // set 'Result' to the contents of the stream
    cv::imwrite("..//Results//foundMarkers.jpg",img);
#endif


    Mat img_gray;

    Mat kernel = (Mat_<uchar>(3,3) << 1,1,1,1,1,1,1,1,1);
    //    Scalar redUpper = CV_RGB(255,255,255);
    //    Scalar redLower = CV_RGB(0,20,20);
    //    inRange(img,redLower,redUpper,img_gray);
    cvtColor(img, img_gray, CV_RGB2GRAY);
    threshold(img_gray, img_gray, 10, 255, 0);

    //Draw grid to seperate connecting circles


    erode(img_gray,img_gray,kernel,Point(-1,-1),3,0,morphologyDefaultBorderValue());

    dilate(img_gray,img_gray,kernel,Point(-1,-1),1,0,morphologyDefaultBorderValue());

    morphologyEx(img_gray,img_gray,MORPH_CLOSE,kernel,Point(-1,-1),1,0,morphologyDefaultBorderValue());

    findContours( img_gray,contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    //printf("Number of Circles Maybe: %i",contours.size());

    forThePaper = Mat::zeros(img_gray.size(), CV_8UC3);

    //fitImageToScreen("Img_Grey 2", contours);

    for(int i = 0; i<contours.size(); i++)
    {
        int blobSize = 0;

        Mat blobOfInterest = Mat::zeros(img_gray.size(), CV_8UC3);

        drawContours(blobOfInterest,contours,i,CV_RGB(255,255,255),CV_FILLED,8,noArray(),0,Point(0,0));

#ifdef DEBUG
        circle(test, centroid(contours[i]),1,CV_RGB(0,255,0),5,8,0);					//Draw the centroids of the markers
#endif
        for (int i = 0; i < blobOfInterest.cols; i++)
        {
            for (int j = 0; j < blobOfInterest.rows; j++)
            {
                Vec3b &intensity = blobOfInterest.at<Vec3b>(j, i);

                if(intensity.val[0] == 255 && intensity.val[1] == 255 && intensity.val[2] == 255)
                    blobSize++;



            }
        }

        float ratio =(((float)(blobSize)/(float)(averageGridSize))*100);

//                imshow( "blobOfInterest", blobOfInterest);                   // Show our image inside it.
//        printf("\n\n---->SIZE = %i", blobSize);
//        printf("\n---->ratio = %f", ratio);
//        printf("\n\n");

//        waitKey(0);
        drawContours(forThePaper,contours,i,CV_RGB(255,255,255),CV_FILLED,8,noArray(),0,Point(0,0));



        if(ratio>=10)

        markerCentroids.push_back(centroid(contours[i]));

    }
#ifdef DEBUG

    //fitImageToScreen("Red Circles Threshold",forThePaper);
    // imwrite("red_circles_threshold.jpg",img);
    //waitKey(100);
    cv::imwrite("..//Results//test.jpg",test);
#endif

}

void RedMarkers::printOffResults()
{
    //-----------------Final Bingo Information-----------------------
    // Test to see what numbers have been called
    printf("\nNumber of called numbers: %i\n",calledNumbers.size());
    for(int i = 0; i<calledNumbers.size(); i++)
        printf("%i ",calledNumbers[i]);
    printf("\n");


    //Test to see what numbers were marked wrong
    printf("\nNumber marked incorrectly: ",wrongNumbersMarked.size());
    for(int i = 0; i<wrongNumbersMarked.size(); i++)
        printf("%i ",wrongNumbersMarked[i]);
    printf("\n");

    //Test to see what numbers were marked wrong
    printf("\nNumbers missing: ");
    for(int i = 0; i<missingNumbers.size(); i++)
        printf("%i ",missingNumbers[i]);
    printf("\n");
}

int RedMarkers::checkNumbers()
{
    for(int i = 0; i < markerCentroids.size();i++)
    {
        int idxCloseSquare = closestSquare(markerCentroids[i]);
        markedNumbers.push_back(currentCardNumbers[idxCloseSquare]);
        circle( forThePaper ,cardGridCentroids[idxCloseSquare],5,CV_RGB(0,255,0),CV_FILLED,8,0);
    }
#ifdef DEBUG
    time_t rawtime;
    struct tm * timeinfo;

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    std::string imageResult;       		   // string which will contain the result
    std::ostringstream imageConvert;  			   // stream used for the conversion
    imageConvert << asctime(timeinfo);     // insert the textual representation of 'Number' in the characters in the stream
    imageResult = imageConvert.str(); // set 'Result' to the contents of the stream
    cv::imwrite(("..//Results//forThePaper" + imageResult + ".jpg").c_str(),forThePaper);
#endif

    //get all the missing numbers
    for(int a = 0; a < currentCardNumbers.size(); a++){
        int current = currentCardNumbers[a];
        //check if the number was called and check if the number has not been marked
        if(contains(current,calledNumbers) && !contains(current,markedNumbers)){
            missingNumbers.push_back(currentCardNumbers[a]);
        }
        //Check if the number was not called and was marked on the card
        if(!contains(current,calledNumbers) && contains(current,markedNumbers)){
            if(currentCardNumbers[a]!=0)
                wrongNumbersMarked.push_back(currentCardNumbers[a]);
        }

    }

    //Sort the vectors with missing and wrong numbers marked
    sort(wrongNumbersMarked.begin(),wrongNumbersMarked.end());
    sort(missingNumbers.begin(),missingNumbers.end());

    for(int i=0; i<25; i++)
    {
        for(int j=0; j<calledNumbers.size(); j++)
        {
            if(currentCardNumbers[i] == calledNumbers[j])
                checkBingoCard[i] = 1;
        }
    }

    if (gameType == 0) {

        //coln 1
        if(checkBingoCard[0]==1 &&checkBingoCard[5]==1&&checkBingoCard[10]==1&&checkBingoCard[15]==1&&checkBingoCard[20]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[20], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //coln 2
        if(checkBingoCard[1]==1 &&checkBingoCard[6]==1&&checkBingoCard[11]==1&&checkBingoCard[16]==1&&checkBingoCard[21]==1)
        {
            //line( cardColor, cardGridCentroids[1], cardGridCentroids[21], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //coln 3
        if(checkBingoCard[2]==1 &&checkBingoCard[7]==1&&checkBingoCard[12]==1&&checkBingoCard[17]==1&&checkBingoCard[22]==1)
        {
            //line( cardColor, cardGridCentroids[2], cardGridCentroids[22], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //coln 4
        if(checkBingoCard[3]==1 &&checkBingoCard[8]==1&&checkBingoCard[13]==1&&checkBingoCard[18]==1&&checkBingoCard[23]==1)
        {
            //line( cardColor, cardGridCentroids[3], cardGridCentroids[23], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //coln 5
        if(checkBingoCard[4]==1 &&checkBingoCard[9]==1&&checkBingoCard[14]==1&&checkBingoCard[19]==1&&checkBingoCard[24]==1)
        {
            //line( cardColor, cardGridCentroids[4], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //row 1
        if(checkBingoCard[0]==1 &&checkBingoCard[1]==1&&checkBingoCard[2]==1&&checkBingoCard[3]==1&&checkBingoCard[4]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[4], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //row 2
        if(checkBingoCard[5]==1 &&checkBingoCard[6]==1&&checkBingoCard[7]==1&&checkBingoCard[8]==1&&checkBingoCard[9]==1)
        {
            //line( cardColor, cardGridCentroids[5], cardGridCentroids[9], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //row 3
        if(checkBingoCard[10]==1 &&checkBingoCard[11]==1&&checkBingoCard[12]==1&&checkBingoCard[13]==1&&checkBingoCard[14]==1)
        {
            //line( cardColor, cardGridCentroids[10], cardGridCentroids[14], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //row 4
        if(checkBingoCard[15]==1 &&checkBingoCard[16]==1&&checkBingoCard[17]==1&&checkBingoCard[18]==1&&checkBingoCard[19]==1)
        {
            //line( cardColor, cardGridCentroids[15], cardGridCentroids[19], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //row 5
        if(checkBingoCard[20]==1 &&checkBingoCard[21]==1&&checkBingoCard[22]==1&&checkBingoCard[23]==1&&checkBingoCard[24]==1)
        {
            //line( cardColor, cardGridCentroids[20], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //X 1
        if(checkBingoCard[0]==1 &&checkBingoCard[6]==1&&checkBingoCard[12]==1&&checkBingoCard[18]==1&&checkBingoCard[24]==1)
        {
            // line( cardColor, cardGridCentroids[0], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            return 1;
        }
        //X 2
        if(checkBingoCard[4]==1 &&checkBingoCard[8]==1&&checkBingoCard[12]==1&&checkBingoCard[16]==1&&checkBingoCard[20]==1)
        {
            //line( cardColor, cardGridCentroids[4], cardGridCentroids[20], Scalar(255, 255, 0), 15 );
            return 1;
        }

        else
            return 0;

    }
    //box
    else if (gameType ==1) {

        int boxLineCount = 0;
        //coln 1
        if(checkBingoCard[0]==1 &&checkBingoCard[5]==1&&checkBingoCard[10]==1&&checkBingoCard[15]==1&&checkBingoCard[20]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[20], Scalar(255, 255, 0), 15 );
            boxLineCount = boxLineCount +1;
        }
        //coln 5
        if(checkBingoCard[4]==1 &&checkBingoCard[9]==1&&checkBingoCard[14]==1&&checkBingoCard[19]==1&&checkBingoCard[24]==1)
        {
            //line( cardColor, cardGridCentroids[4], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            boxLineCount = boxLineCount +1;
        }
        //row 1
        if(checkBingoCard[0]==1 &&checkBingoCard[1]==1&&checkBingoCard[2]==1&&checkBingoCard[3]==1&&checkBingoCard[4]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[4], Scalar(255, 255, 0), 15 );
            boxLineCount = boxLineCount +1;
        }
        //row 5
        if(checkBingoCard[20]==1 &&checkBingoCard[21]==1&&checkBingoCard[22]==1&&checkBingoCard[23]==1&&checkBingoCard[24]==1)
        {
            //line( cardColor, cardGridCentroids[20], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            boxLineCount = boxLineCount +1;
        }

        if (boxLineCount==4) {
            return 1;
        }
        else
            return 0;


    }
    //cross
    else if (gameType == 2) {
        int crossCounter = 0;
        //X 1
        if(checkBingoCard[0]==1 &&checkBingoCard[6]==1&&checkBingoCard[12]==1&&checkBingoCard[18]==1&&checkBingoCard[24]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[24], Scalar(255, 255, 0), 15 );
            crossCounter = crossCounter +1;
        }
        //X 2
        if(checkBingoCard[4]==1 &&checkBingoCard[8]==1&&checkBingoCard[12]==1&&checkBingoCard[16]==1&&checkBingoCard[20]==1)
        {
            //line( cardColor, cardGridCentroids[4], cardGridCentroids[20], Scalar(255, 255, 0), 15 );
            crossCounter = crossCounter +1;
        }

        if (crossCounter ==2) {
            return 1;
        }
        else
            return 0;

    }
    //corners
    else if (gameType == 3) {
        //coln 1
        if(checkBingoCard[0]==1 &&checkBingoCard[24]==1&&checkBingoCard[4]==1&&checkBingoCard[20]==1)
        {
            //line( cardColor, cardGridCentroids[0], cardGridCentroids[20], Scalar(255, 255, 0), 15 );
            return 1;
        }
        else
            return 0;


    }
    //full
    else if (gameType == 4) {
        //coln 1
        int bingoCardFullCounter = 0;
        for (int i =0; i<=24; i++) {
            if (checkBingoCard[i]) {
                bingoCardFullCounter++;
            }
        }
        if (bingoCardFullCounter!=25)
            return 0;
        else
            return 1;

    }

}
