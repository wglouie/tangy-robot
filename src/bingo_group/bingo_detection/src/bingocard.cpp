/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetection Code
 *
 * January 30, 2015
 */
#include "bingocard.h"

#define DEBUG
template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

bool gridSortYTopToBottomValues(const Point2f& i, const Point2f& j)
{
    /* Sort top to bottom */
    if (i.y<j.y)
        return true;
    else
        return false;
}
bool gridSortXLeftToRightValues(const Point2f& i, const Point2f& j)
{
    /* Sort from left to right*/
    if (i.x<j.x)
        return true;
    else
        return false;
}

bool gridSortYBottomToTopValues(const Point2f& i, const Point2f& j)
{
    /* Sortbottom to top */
    if (i.y>j.y)
        return true;
    else
        return false;
}
bool gridSortXRightToLeftValues(const Point2f& i, const Point2f& j)
{
    /* Sort from right to left */
    if (i.x>j.x)
        return true;
    else
        return false;
}

bool cardGridSortCmpTop(const Point2f& i, const Point2f& j)
{
    /* Sort from left to right, top to bottom */
    if (i.y-j.y<10 && i.y-j.y>-10)
        return i.x < j.x;
    else
        return i.y < j.y;

}
bool cardGridSortCmpLeft(const Point2f& i, const Point2f& j)
{
    /* Sort from bottom to top, left to right */
    if (i.x-j.x<10 && i.x-j.x>-10)
        return i.y > j.y;
    else
        return i.x < j.x;

}
bool cardGridSortCmpRight(const Point2f& i, const Point2f& j)
{
    /* Sort from top to bottom, right to left */
    if (i.x-j.x<10 && i.x-j.x>-10)
        return i.y < j.y;
    else
        return i.x > j.x;

}
bool cardGridSortCmpBottom(const Point2f& i, const Point2f& j)
{
    /* Sort from right to left, bottom to top */
    if (i.y-j.y<10 && i.y-j.y>-10)
        return i.x > j.x;
    else
        return i.y > j.y;

}

BingoCard::BingoCard(Mat inputImage,
                     string inputCardFolder,
                     vector<int> inputCalledNumbers,
                     int inputGameType,
                     string inputNumberDatabaseFile):
    numberDatabaseFile(inputNumberDatabaseFile),
    cardFolder(inputCardFolder),
    calledNumbers(inputCalledNumbers),
    gameType(inputGameType),
    imageProcessed(inputImage),
    cardBingo{-1},
    missingNumbers(0),
    wrongNumbers(0),
    cardGridCentroids(0)
{
    //WILL put in one more object in before checkCard()
    //This object will find how many cards it sees and checks for the biggest one

    publisher = n.advertise<std_msgs::String>("/bingoDetectionInfo", 1000);


    if(imageProcessed.getCentroids().size()!=26){

        ss << "\nNo Grid Found\n";
        cardBingo=-1;
    }

    else
    {

        ss <<"\n\n----->Grid Found<-----\n\n";

        cardGridCentroids=imageProcessed.getCentroids();


        ss << "Last centroid before popping back ratio to image size: " << cardGridCentroids[25].x<< cardGridCentroids[25].y;
        std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),cardGridSortCmpTop);
        cardGridCentroids.erase (cardGridCentroids.begin()+12);

//#ifdef DEBUG
//        std::ofstream resultFile;
//        resultFile.open ("//bingoTestResults//BingoDetectionResults.txt",std::ofstream::app);
//        std::ostringstream convert;
//        convert << "\nCentroid Sorted: ";

//        int counter = 1;
//        for(int i = 0; i<cardGridCentroids.size(); i++)
//        {
//            convert << "\nCentroid "<< counter <<" :"  << cardGridCentroids[i] << "\n";
//            counter++;
//        }
//        resultFile << convert.str(); // set 'resultFile' to the contents of the stream
//        resultFile.close();

//#endif
        //cardGridCentroids.pop_back();

        checkCard();
        ss << "\n\n----->Checking Done<-----\n\n";



    }
    //cardMarkerCheck = NULL;
    //redMarkersCheck = NULL;
    msg.data = ss.str();
    publisher.publish(msg);
}

BingoCard::~BingoCard()
{
    // delete cardMarkerCheck;
    // delete redMarkersCheck;
}


//Perform SURF Detection, then straighten out the image then check the red markers
//If no match return -1 for has bingo
//If match and no bingo return 0
//If match and Bingo return 1
void BingoCard::checkCard()
{
    //perform SURF detection on card
    SURFDetection cardMarkerCheck(cardFolder, imageProcessed.getUndistortedImage());

    ss << "\nOrientaion of card  " << cardMarkerCheck.getOrientation() << "\n";

    if(cardMarkerCheck.getOrientation()!=-1)
    {


        //Orientation editing done in here
        if(cardMarkerCheck.getOrientation() == 0){

            std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),gridSortYTopToBottomValues);
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end()-20,gridSortXLeftToRightValues);
            std::sort(cardGridCentroids.begin()+5,cardGridCentroids.end()-15,gridSortXLeftToRightValues);
            std::sort(cardGridCentroids.begin()+10,cardGridCentroids.end()-10,gridSortXLeftToRightValues);
            std::sort(cardGridCentroids.begin()+15,cardGridCentroids.end()-5,gridSortXLeftToRightValues);
            std::sort(cardGridCentroids.begin()+20,cardGridCentroids.end(),gridSortXLeftToRightValues);

        }
        if(cardMarkerCheck.getOrientation() == 1)
        {
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),gridSortXRightToLeftValues);
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end()-20,gridSortYTopToBottomValues);
            std::sort(cardGridCentroids.begin()+5,cardGridCentroids.end()-15,gridSortYTopToBottomValues);
            std::sort(cardGridCentroids.begin()+10,cardGridCentroids.end()-10,gridSortYTopToBottomValues);
            std::sort(cardGridCentroids.begin()+15,cardGridCentroids.end()-5,gridSortYTopToBottomValues);
            std::sort(cardGridCentroids.begin()+20,cardGridCentroids.end(),gridSortYTopToBottomValues);


        }

        if(cardMarkerCheck.getOrientation() == 2)
        {
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),gridSortYBottomToTopValues);
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end()-20,gridSortXRightToLeftValues);
            std::sort(cardGridCentroids.begin()+5,cardGridCentroids.end()-15,gridSortXRightToLeftValues);
            std::sort(cardGridCentroids.begin()+10,cardGridCentroids.end()-10,gridSortXRightToLeftValues);
            std::sort(cardGridCentroids.begin()+15,cardGridCentroids.end()-5,gridSortXRightToLeftValues);
            std::sort(cardGridCentroids.begin()+20,cardGridCentroids.end(),gridSortXRightToLeftValues);



        }

        if(cardMarkerCheck.getOrientation() == 3)
        {
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end(),gridSortXLeftToRightValues);
            std::sort(cardGridCentroids.begin(),cardGridCentroids.end()-20,gridSortYBottomToTopValues);
            std::sort(cardGridCentroids.begin()+5,cardGridCentroids.end()-15,gridSortYBottomToTopValues);
            std::sort(cardGridCentroids.begin()+10,cardGridCentroids.end()-10,gridSortYBottomToTopValues);
            std::sort(cardGridCentroids.begin()+15,cardGridCentroids.end()-5,gridSortYBottomToTopValues);
            std::sort(cardGridCentroids.begin()+20,cardGridCentroids.end(),gridSortYBottomToTopValues);


        }



        Mat processedImg = imageProcessed.getBigGrid();
        int marker = cardMarkerCheck.getMatchedMarker();
        ss << "\n\nMarkerMatched ------> " << marker << "\n\n";

        //Check red markers on card for Bingo
        RedMarkers redMarkersCheck(processedImg,
                                   calledNumbers,
                                   gameType,
                                   numberDatabaseFile,
                                   marker,
                                   cardGridCentroids);



        wrongNumbers=redMarkersCheck.getWrongNumbersMarked();
        missingNumbers=redMarkersCheck.getMissingNumbers();
        cardBingo = redMarkersCheck.checkForBingo();
        //redMarkersCheck->printOffResults();

        //Called numbers
        ss << "Called Numbers: ";
        for(int i = 0; i<calledNumbers.size(); i++)
            ss << calledNumbers[i] << ", ";
        ss << "\n";

        //Numbers Marked Incorrectly
        ss << "Numbers marked incorrectly: ";
        for(int i = 0; i<wrongNumbers.size(); i++)
            ss << wrongNumbers[i] << ", ";
        ss << "\n";

        //Missing Number
        ss << "Numbers missing: ";
        for(int i = 0; i<missingNumbers.size(); i++)
            ss << missingNumbers[i] << ", ";
    }
    else
        cardBingo = -1; //ERROR
}

int BingoCard::cardHasBingo()
{
    return cardBingo;
}

vector<int> BingoCard::getMissingNumbers()
{
    return missingNumbers;
}

vector<int> BingoCard::getWrongNumbers()
{
    return wrongNumbers;
}
