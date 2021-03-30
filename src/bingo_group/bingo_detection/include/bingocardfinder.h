/*
 * Frank Despond
 *
 * Version 1.0
 *
 * January 28, 2015
 */
#ifndef BINGOCARDFINDER_H
#define BINGOCARDFINDER_H

//IDEA-----> Have a vector storing the Mats ranging from largest to smallest eg.. bingoCards[0] (biggest) -bingoCards[n] (smallest)
//IDEA----->Have the image reduced in this part instead of further down the road

using namespace cv;
using namespace std;

class BingoCardFinder : private SquareFinder
{
public:
    BingoCardFinder();
    ~BingoCardFinder();

    Mat getLargestBingoCardImage();
private:
    Mat inputImage;
    Mat outputImage;

};

#endif // BINGOCARDFINDER_H
