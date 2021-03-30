#ifndef DEBUGOPENCV_H
#define DEBUGOPENCV_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/photo/photo.hpp"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

class debugOpenCV
{
public:
    debugOpenCV(bool debug);
    ~debugOpenCV();

    void imageToScreen( char* windowName, Mat src, int lineNumber,  char* file);
private:
    bool debug;
};

#endif // DEBUGOPENCV_H
