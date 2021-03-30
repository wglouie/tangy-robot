#include "debugopencv.h"

debugOpenCV::debugOpenCV(bool inputDebug)
{
    debug = inputDebug;
}

debugOpenCV::~debugOpenCV()
{

}

void debugOpenCV::imageToScreen(char *windowName, Mat src, int lineNumber, char *file)
{
    if(debug){
    Mat temp;
    resize(src,temp,Size(640,480),0,0,INTER_LINEAR);
    imshow(windowName,temp);
    waitKey(10);
    printf("\n----------Debugging View Image-------\n\nFile: %s\nCode line: %i\n-------------------------", file ,lineNumber);
    }
    else
    {
    }
}
