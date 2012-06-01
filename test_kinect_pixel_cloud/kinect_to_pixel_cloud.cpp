 /*		Graphics - Assignment 1 - Part 1 - A
  *		Draw a single roating cube
  *
  *		Jordan Stadler
  *		100366066
  *		JAN 2011     
  */
 

#include <stdlib.h>

#include <iostream>
#include <stdexcept>

//#include <GL/gl.h>
//#include <GL/glu.h>
#include <glut.h>
#include <math.h>

// include opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//using namespace cv;

#include <XnCppWrapper.h>
using namespace xn;
#define SAMPLE_XML_PATH "./SamplesConfig.xml"
 
 int window; 	//id of the window
 
 /*
  *  Display function.  This function is called each time the
  *  display screen needs to be redrawn
  */
 float angle = 0.0;

 float xPos = 0.0;
 float yPos = 0.0;
 float zPos = 0.0;

 double angleX = 0.0;
double angleY = 0.0;

double lx = 0.0, ly = 0.0, lz = 0.0;

double x = 0.0, y = 0.0, z = 20.0;

double deltaAngleX = 0.0;
double deltaAngleY = 0.0;
double deltaMove = 0.0;
int xOrigin = -1;
int yOrigin = -1;

cv::Size image_size;
//cv::Mat A(443,602, CV_8UC1);
cv::Mat A;

cv::Mat m_depth16u( 480, 640, CV_16UC1 );
cv::Mat m_depthImage( 480, 640, CV_8UC1 );

Context g_context;
DepthGenerator g_depth;
DepthMetaData g_depthMD;

/*
 * void keyMove (double deltaMove );
 *
 * 
 */ 
void keyMove (double deltaMove ) {
	x += deltaMove * lx * 0.1;
	y += deltaMove * ly * 0.1;
	z += deltaMove * lz * 0.1;
}

 void display() {

	 g_depth.GetMetaData( g_depthMD );
	 memcpy( m_depth16u.data, g_depthMD.Data(), 640*480*2 );
	 m_depth16u.convertTo( m_depthImage, CV_8U, 255/2096.0 );

	 if(deltaMove) {
		keyMove(deltaMove);
	}
	  
	  //glMatrixMode(GL_MODELVIEW);
	 // gluLookAt(x, y, z, x+lx, y+ly, z+lz, 0.0, 1.0, 0.0);
	// Note: Lighting has been added directly from example four.c so as
	// to demonstrate that the sphere is rotating, otherwise it just 
	// appears to be a circle
 
	//GLfloat ambient[4] = { 0.0, 0.0, 0.4, 1.0 };// blue ambient reflection
	//GLfloat specular[4] = { 1.1, 0.8, 0.1, 1.0 };// green specular reflection

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//gluLookAt(0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	 gluLookAt(x, y, z, x+lx, y+ly, z+lz, 0.0, 1.0, 0.0);
	
	glColor3f (1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	 //printf("image w: %d h: %d\n", image_size.width, image_size.height);
	 for( int l = 0; l < m_depthImage.rows; l++ ) {
		 for( int m = 0; m < m_depthImage.cols; m++) {
			 //printf("dims: %d\n", A.dims);
			 //printf("element: %d\n", (int)A.at<uchar>(l,m) );
			 glVertex3f( m*0.01, 1-l*0.01, -((int)m_depthImage.at<uchar>(l,m)) *0.01 );
		 }
	 }
	 glEnd();

	 /*
	glColor3f (1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	for(double i = -1; i <= 1; i+=0.01){
		for(double j = -1; j <= 1; j+=0.01){
			//for(double k = -1; k <= 1; k+=0.01){
				glVertex3f( i, j, 0);
			//}
		}
	}
	glEnd();*/
	glutSwapBuffers();
}

/*
 *  The animate function is an idle function that is called whenever
 *  Glut isn't busy doing something else.  All this function does is
 *  cause the window to be redrawn.
 */
void animate() {
     
     // We must set the current window, since a window isn't
     // set before this function is called
     glutSetWindow(window);
     //  Ask Glut to redisplay the current window
     glutPostRedisplay();
     angle = angle + 0.1; 
}
     

/* gfxinit.  The function initializes OpenGL and prepares it for
 * drawing.  This function is called once at the start of the
 * program.
 */

void gfxinit() {
	try{
	XnMapOutputMode mapMode;
        mapMode.nXRes = 640;
        mapMode.nYRes = 480;
        mapMode.nFPS = 30;
        XnStatus rc;
		EnumerationErrors errors;
        rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
		if ( rc == XN_STATUS_NO_NODE_PRESENT ) {
            XnChar strError[1024];
            errors.ToString( strError, 1024 );
            throw std::runtime_error( strError );
        }
        else if( rc != XN_STATUS_OK ) {
            std::cout << "Open failed: ";
            throw std::runtime_error( xnGetStatusString( rc ) );
        }
 
        std::cout << "Open success." << std::endl;

		//----------------------------------------
        rc = g_context.FindExistingNode( XN_NODE_TYPE_DEPTH, g_depth );
        if( rc != XN_STATUS_OK ) {
            std::cout << "Get depth error: ";
            throw std::runtime_error( xnGetStatusString( rc ) );
        }
		//----------------------------------------
		rc = g_depth.SetMapOutputMode( mapMode );
        if( rc != XN_STATUS_OK ) {
            std::cout << "Set depth output mode error: ";
            throw std::runtime_error( xnGetStatusString( rc ) );
        }
	}catch( std::exception& ex ) {
		std::cout << ex.what() << std::endl;
	}
	/*
	A = cv::imread("stadler.png", 0);

	do{
		cv::imshow("image", A);
	}while(cv::waitKey(5) != 'q');
	
	image_size = A.size();
     */
	 GLfloat lightpos[4] = { 0.0, 5.0, 0.0, 1.0 };     // light position
	 GLfloat lightamb[4] = { 0.3, 0.3, 0.3, 1.0 };     // ambient colour
	 GLfloat lightdif[4] = { 0.8, 0.8, 0.8, 1.0 };     // diffuse colour
	 GLfloat lightspec[4] = { 0.6, 0.6, 0.6, 1.0 };    // specular colour

     // enable the depth buffer
     glEnable(GL_DEPTH_TEST);
     // set the projection to perspective
     glMatrixMode(GL_PROJECTION);
     gluPerspective(40.0, 1.0, 1.0, 100.0);
     glMatrixMode(GL_MODELVIEW);
     gluLookAt(0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
     glShadeModel(GL_SMOOTH);
     // set the light position, 5 units along the y axis
     glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
     // set the ambient light colour
     glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
     // set the diffuse light colour
     glLightfv(GL_LIGHT0, GL_DIFFUSE, lightdif);
     // set the specular light colour
     glLightfv(GL_LIGHT0, GL_SPECULAR,lightspec);
     // turn on lighting
     glEnable(GL_LIGHTING);
     // enable light 0, all the other lights are off
     glEnable(GL_LIGHT0);     
}



// Control global axis rotation
void key_special(int key, int x, int y) {
   switch (key) {
	  case GLUT_KEY_UP:
		  yPos += 1;
		  break;
	  case GLUT_KEY_DOWN:
		  yPos -= 1;
		  break;
	  case GLUT_KEY_LEFT:
		  xPos -= 1;
		  break;
	  case GLUT_KEY_RIGHT:
		  xPos += 1;
		  break;
   }
   glutPostRedisplay();
}

// Exit function
void key_check(unsigned char key, int x, int y) {
   switch (key) {
      case 27: // Escape
	     exit(0);
		 break;
	 case 119: // [w] forward
		deltaMove = 0.5;
		break;
	case 115: // [s] back
		deltaMove = -0.5;
		break;
   }
   glutPostRedisplay();
}

/*
 * void key_release( unisgned char key, int x, int y);
 *
 * when the key is released, stop moving in that direction 
 */
void key_release( unsigned char key, int x, int y) {
	switch(key) {
		case 119: // [w] forward
			deltaMove = 0.0;
			break;
		case 115: // [s] back
			deltaMove = 0.0;
			break;
	}	
}

/*
 * void mouse_move( int x, int y);
 *
 * Detects the change is mouse direction
 */
void mouse_move( int x, int y) {
	if( xOrigin >= 0) {
		deltaAngleX = ( x - xOrigin) * 0.02;
		deltaAngleY = ( y - yOrigin) * 0.02;

		lx = sin(angleX + deltaAngleX);
		ly = -1*sin(angleY + deltaAngleY);
		lz = -1*cos(angleX + deltaAngleX);
	}
}

/*
 * void mouse_button( int button, int state, int x, int y);
 *
 * Handles mouse button presses
 */
void mouse_button( int button, int state, int x, int y) {
	if(button == GLUT_LEFT_BUTTON) {
		if( state == GLUT_UP) {
			angleX += deltaAngleX;
			angleY += deltaAngleY;
			xOrigin = -1;
			yOrigin = -1;
		}else{
			xOrigin = x;
			yOrigin = y;
		}
	}
}

/*
 * The main function
 */
 
int main(int argc, char **argv) {

     glutInit(&argc,argv);
	 glutInitWindowSize(1280,960);
     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

     window = glutCreateWindow("Pixel Cloud");

     glutDisplayFunc(display);

     glutIdleFunc(animate);

     gfxinit();
  	 glutKeyboardFunc(key_check);
	 glutKeyboardUpFunc(key_release);
	 glutMouseFunc(mouse_button);
	 glutMotionFunc(mouse_move);
  	 //glutSpecialFunc(key_special);

     glutMainLoop();
     
}
