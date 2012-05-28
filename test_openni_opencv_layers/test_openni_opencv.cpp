/*
 * Simple test program from http://pebblerchung.wikispaces.com/OpenCV-OpenNI
 *
 */ 

#include <iostream>
#include <stdexcept>
using namespace std;
 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
 
//#include <ni/XnCppWrapper.h>
#include <XnCppWrapper.h>
using namespace xn;
#define SAMPLE_XML_PATH "./SamplesConfig.xml"
 
int main()
{
    Context g_context;
    try {
        XnStatus rc;
        EnumerationErrors errors;
        rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
        if ( rc == XN_STATUS_NO_NODE_PRESENT ) {
            XnChar strError[1024];
            errors.ToString( strError, 1024 );
            throw runtime_error( strError );
        }
        else if( rc != XN_STATUS_OK ) {
            cout << "Open failed: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
 
        cout << "Open success." << endl;
 
        //------------------------------
        DepthGenerator g_depth;
        rc = g_context.FindExistingNode( XN_NODE_TYPE_DEPTH, g_depth );
        if( rc != XN_STATUS_OK ) {
            cout << "Get depth error: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
 
        //------------------------------
        ImageGenerator g_image;
        rc = g_context.FindExistingNode( XN_NODE_TYPE_IMAGE, g_image );
        if( rc != XN_STATUS_OK ) {
            cout << "Get image error: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
 
        cout << "Generate success." << endl;
 
        //------------------------------
        DepthMetaData g_depthMD;
        ImageMetaData g_imageMD;
        g_depth.GetMetaData( g_depthMD );
        g_image.GetMetaData( g_imageMD );
 
        if( g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes() ) {
            throw std::runtime_error( "The device depth and image resolution must be equal!" );
        }
        if( g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24 ) {
            throw std::runtime_error( "The device image format must be RGB24" );
        }
 
        cout << "Get meta data success." << endl;
 
        //------------------------------
        XnMapOutputMode mapMode;
        mapMode.nXRes = 640;
        mapMode.nYRes = 480;
        mapMode.nFPS = 30;
        rc = g_depth.SetMapOutputMode( mapMode );
        if( rc != XN_STATUS_OK ) {
            cout << "Set depth output mode error: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
        rc = g_image.SetMapOutputMode( mapMode );
        if( rc != XN_STATUS_OK ) {
            cout << "Set image output mode error: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
 
        cout << "Set output mode success." << endl << "Starting Generate.." << endl;
 
        //------------------------------
        rc = g_context.StartGeneratingAll();
        if( rc != XN_STATUS_OK ) {
            cout << "Starting Kinect error: ";
            throw runtime_error( xnGetStatusString( rc ) );
        }
 
        //------------------------------
        Mat  m_depth16u( 480, 640, CV_16UC1 );
        Mat  m_rgb8u( 480, 640, CV_8UC3 );
        Mat  m_DepthShow( 480, 640, CV_8UC1 );
        Mat  m_ImageShow( 480, 640, CV_8UC3 );
        Mat foreground(480, 640, CV_8UC3 );
        Mat midground(480, 640, CV_8UC3 );
        Mat background(480, 640, CV_8UC3 );
        Mat all0(480, 640, CV_8UC1, Scalar(1));
        Mat all1(480, 640, CV_8UC1, Scalar(1)); 
        Mat all85(480, 640, CV_8UC1, Scalar(85));
        Mat all170(480, 640, CV_8UC1, Scalar(170));
        Mat all255(480, 640, CV_8UC1, Scalar(255));
        Mat fgMask;
        Mat mgMask;
        Mat bgMask;
        Mat bg2Mask;
        unsigned char key;
        while( key != 27 && key != 'q' ) {
            rc = g_context.WaitAnyUpdateAll();
            if ( rc != XN_STATUS_OK ) {
                cout << "Read failed: ";
                throw runtime_error( xnGetStatusString( rc ) );
            }
            g_depth.GetMetaData( g_depthMD );
            g_image.GetMetaData( g_imageMD );
            memcpy( m_depth16u.data, g_depthMD.Data(), 640*480*2 );
            memcpy( m_rgb8u.data, g_imageMD.Data(), 640*480*3 );
            m_depth16u.convertTo( m_DepthShow, CV_8U, 255/2096.0 );
            cvtColor( m_rgb8u, m_ImageShow, CV_RGB2BGR );
            inRange(m_DepthShow, all0, all1, bg2Mask);
            inRange(m_DepthShow, all1, all85, fgMask);
            inRange(m_DepthShow, all85, all170, mgMask);
            inRange(m_DepthShow, all170, all255, bgMask);
            
            bgMask += bg2Mask;

            m_ImageShow.copyTo(foreground, fgMask);
            m_ImageShow.copyTo(midground, mgMask);
            m_ImageShow.copyTo(background, bgMask);

            imshow( "depth", m_DepthShow );
            imshow( "image", m_ImageShow );
            imshow( "foreground", foreground);
            imshow( "midground", midground);
            imshow( "background", background);
            foreground = all0;
            midground = all0;
            background = all0;


            // shows color layers
            /*
            imshow( "0", all0);
            imshow( "85", all85);
            imshow( "170", all170);
            imshow( "255", all255);
            */
            key = waitKey( 10 );
        }
    }
    catch( exception& ex ) {
        cout << ex.what() << endl;
    }
    g_context.StopGeneratingAll();
    g_context.Shutdown();
 
    return 0;
}
