/**
 * How to Play AVI Files with OpenCV
 *
 * Author  Nash
 * License GPL
 * Website http://nashruddin.com
 */

#include <stdio.h>
#include "cv.h"
#include "highgui.h"

int main( int argc, char** argv )
{
	IplImage  *frame;
	int 	  key;

	/* supply the AVI file to play */
	assert( argc == 2 );

	/* load the AVI file */
	CvCapture *capture = cvCaptureFromAVI( argv[1] );

	/* always check */
	if( !capture ) return 1;

	int fps = ( int )cvGetCaptureProperty( capture, CV_CAP_PROP_FPS );

	/* display video */
	cvNamedWindow( "video", 0 );

	while( key != 'q' ) {
		/* get a frame */
		frame = cvQueryFrame( capture );

		/* always check */
		if( !frame ) break;

		/* display frame */
		cvShowImage( "video", frame );

		/* quit if user press 'q' */
		//key = cvWaitKey( 1000 / fps );
		key = cvWaitKey( 50 );
	}

	/* free memory */
	cvReleaseCapture( &capture );
	cvDestroyWindow( "video" );

	return 0;
}
