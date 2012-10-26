#include "main.h"

void display()
{
	// (7) Rendering
	gFingertipPoseEstimation.TickCountBegin();

	// display info
	IplImage *image = gFingertipPoseEstimation.OnDisplay();

	// check if there have been any openGL problems
	GLenum errCode = glGetError();
	if(errCode != GL_NO_ERROR) {
		const GLubyte *errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL error: %s\n", errString);
	}

	// clear the buffers of the last frame
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	// set the drawing region of the window
	glViewport(0, 0, image->width, image->height);

	// set viewing frustrum to match camera FOV (ref: ARTag's 3d_augmentations.cpp)
	gFingertipPoseEstimation.SetOpenGLFrustrum();

	// set modelview matrix of the camera
	gFingertipPoseEstimation.SetOpenGLModelView();

/*
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(gFingertipPoseEstimation.QueryModelViewMat());
*/

	// render virtual objects
	GLfloat colorRed[3] = {1.0, 0.0, 0.0};
	GLfloat colorGreen[3] = {0.0, 1.0, 0.0};
	GLfloat colorBlue[3] = {0.0, 0.0, 1.0};
	GLfloat colorYellow[3] = {1.0, 1.0, 0.0};
	GLfloat colorDarkYellow[3] = {0.6, 0.6, 0.0};
	GLfloat colorWhite[3] = {1.0, 1.0, 1.0};
	GLfloat colorGray[3] = {0.5, 0.5, 0.5};


	if (gFingertipPoseEstimation.QueryValidPose()) {
		glPushMatrix();

		if (gnModel == MODEL_BUNNY) {
			glPushMatrix();
			glScalef(1000, 1000, 1000);
			glRotatef(90, 1, 0, 0);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorGray);
			glDrawElements(GL_TRIANGLES, 3 * NUM_TRIANGLES, GL_UNSIGNED_INT, triangles);
			glPopMatrix();
		} else if (gnModel == MODEL_EARTH) {
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, gnEarthTexID);
			GLUquadricObj *quadObj;
			quadObj = gluNewQuadric();
			gluQuadricDrawStyle(quadObj, GLU_FILL);
			gluQuadricTexture(quadObj, GL_TRUE);
			glPushMatrix();
			glTranslatef(0, 0, 50);
			gluSphere(quadObj, 50, 32, 32);
			glPopMatrix();
			glDisable(GL_TEXTURE_2D);
		} else if (gnModel == MODEL_MAGIC) {
			if (frame_num >= N_FRAME)
				frame_num = E_FRAME;
			glEnable(GL_ALPHA_TEST);
			glAlphaFunc(GL_GEQUAL, 0.5);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			mqoCallSequence(mqo_seq, frame_num);
			glPopMatrix();
			glDisable(GL_BLEND);
			glDisable(GL_ALPHA_TEST);
			frame_num++;
		} else if (gnModel == MODEL_COORDINATE_AXES) {
			glDisable(GL_LIGHTING);
			glLineWidth(3);
			glBegin(GL_LINES);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorRed);
			glColor3fv(colorRed);
			glVertex3f(0, 0, 0);
			glVertex3f(100, 0, 0);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorGreen);
			glColor3fv(colorGreen);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 100, 0);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorBlue);
			glColor3fv(colorBlue);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 0, 100);
			glEnd();
			glPushMatrix();
			glColor3fv(colorRed);
			glTranslatef(100, 0, 0);
			glRotatef(90, 0, 1, 0);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glPushMatrix();
			glColor3fv(colorGreen);
			glTranslatef(0, 100, 0);
			glRotatef(-90, 1, 0, 0);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glPushMatrix();
			glColor3fv(colorBlue);
			glTranslatef(0, 0, 100);
			glRotatef(90, 0, 0, 1);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glEnable(GL_LIGHTING);
		}

/*
		glPushMatrix();
		glTranslatef(0, 0, 30);
		glRotatef(90, 1, 0, 0);
		glutSolidTeapot(50);
		glPopMatrix();
*/

		glPopMatrix();
	}

	glutSwapBuffers();

	// Record the output video
	if (fRecord) {
		glReadPixels(0, 0, image->width, image->height, GL_RGB, GL_UNSIGNED_BYTE, image->imageData);
		cvConvertImage(image, image, CV_CVTIMG_SWAP_RB);
		cvWriteFrame(gpRecordOutput, image);
	}

	if (fScreenshot) {
		glReadPixels(0, 0, image->width, image->height, GL_RGB, GL_UNSIGNED_BYTE, image->imageData);
		cvConvertImage(image, image, CV_CVTIMG_SWAP_RB);
		cvSaveImage(SCREENSHOT_FILENAME, image);
		fScreenshot = false;
	}

	// (7) Rendering
	gFingertipPoseEstimation.TickCountEnd();
	gFingertipPoseEstimation.TickCountNewLine();
}

void keyboard_func(unsigned char key, int x, int y)
{
	switch(key) {
		case ' ':
			gFingertipPoseEstimation.Reset();
			break;
		case 'h':
		case 'H':
			fHandRegion = !fHandRegion;
			break;
		case 'd':
		case 'D':
			fDistTrans = !fDistTrans;
			break;
		case 'l':
		case 'L':
			gFingertipPoseEstimation.LoadFingertipCoordinates(gszFingertipFilename);
			break;
		case 's':
		case 'S':
			gFingertipPoseEstimation.SaveFingertipCoordinates(gszFingertipFilename);
			break;
		case 'c':
		case 'C':
			gFingertipPoseEstimation.ToggleChessboard();
			break;
		case 'b':
		case 'B':
			gFingertipPoseEstimation.ToggleBuildHandModel();
			break;
		case 'r':
		case 'R':
			fRecord = !fRecord;
			break;
		case 'p':
		case 'P':
			gFingertipPoseEstimation.PrintScreenshot();
			fScreenshot = true;
			break;
		case 'q':
		case 'Q':
			cvReleaseVideoWriter(&gpRecord);
			cvReleaseVideoWriter(&gpRecordOutput);
			gCapture.Terminate();
			gFingertipPoseEstimation.Terminate();
			exit(0);
			break;
	}
}

void tick_func()
{
	// capture
	IplImage *frame = 0;

	gCapture.CaptureFrame();
	frame = gCapture.QueryFrame();
	if (!frame)
		return;

	gFingertipPoseEstimation.OnCapture(frame, gCapture.QueryTickCount());

	// record
	if (fRecord)
		cvWriteFrame(gpRecord, frame);

	// process fingertip pose estimation
	gFingertipPoseEstimation.OnProcess();

	// draw objects
	glutPostRedisplay();
}

void init()
{
	// turn on z-buffering, so we get proper occlusion
	glEnable(GL_DEPTH_TEST);

	// properly scale normal vectors
	glEnable(GL_NORMALIZE);

	// turn on default lighting
	glEnable(GL_LIGHTING);

	// light 0
	GLfloat light_position[] = {100.0, 500, 200, 1.0};
	GLfloat white_light[] = {1.0, 1.0, 1.0, 0.8};
	GLfloat lmodel_ambient[] = {0.9, 0.9, 0.9, 0.5};

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_TEXTURE_2D);
	glCullFace(GL_BACK);
	glFrontFace(GL_CW);

	// Init Earth Texture
	glGenTextures(1, &gnEarthTexID);
	glBindTexture(GL_TEXTURE_2D, gnEarthTexID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
	IplImage *tempImage = cvLoadImage(EARTH_TEXTURE_FILENAME, 1);

	// convert image R and B channel
	cvConvertImage(tempImage, tempImage, CV_CVTIMG_SWAP_RB);
	cvFlip(tempImage);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tempImage->width, tempImage->height, GL_RGB, GL_UNSIGNED_BYTE, tempImage->imageData);
	cvReleaseImage(&tempImage);

	// Init bunny
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, bunny);
	glNormalPointer(GL_FLOAT, 0, normals);

	// Callback functions
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard_func);
	glutIdleFunc(tick_func);

}

bool parse(int argc, char ** argv)
{
	bool fResult = false;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-model")) {
			// Render Model
			i++;
			if (i == argc)
				goto Finished;

			// take the model option
			if (!strcmp(argv[i], "bunny"))
				gnModel = MODEL_BUNNY;
			else if (!strcmp(argv[i], "earth"))
				gnModel = MODEL_EARTH;
			else if (!strcmp(argv[i], "magic"))
				gnModel = MODEL_MAGIC;
			else
				gnModel = MODEL_COORDINATE_AXES;
		} else if (!strcmp(argv[i], "-coord")) {
			// Fingertip Coordinates Data File
			i++;
			if (i == argc)
				goto Finished;

			// take the fingertip coordinate data file
			strcpy(gszFingertipFilename, argv[i]);
		} else if (!strcmp(argv[i], "-input")) {
			// Input Video File
			i++;
			if (i == argc)
				goto Finished;

			// take the video file
			fInputVideoFile = true;
			strcpy(gszInputVideoFilename, argv[i]);
		} else if (!strcmp(argv[i], "-flip")) {
			// Flip Captured Frames
			fFlipFrame = !fFlipFrame;
		} else {
			goto Finished;
		}
	}
	fResult = true;

Finished:
	if (fResult == false) {
		// Print Usage
		printf("%s [-model bunny|earth|magic] [-coord fingertip.dat] [-input video.avi] [-flip]\n", argv[0]);
		printf("\n");
		printf("-	[-model bunny|earth|magic]: models to render. (Default=coordinate axes)\n");
		printf("-	[-coord fingertip.dat]: fingertip coordinates data file.\n");
		printf("                            (Default=fingertip_640x480.dat)\n");
		printf("-	[-input video.avi]: input video file. (Default=live capture)\n");
		printf("-	[-filp]: flip video images vertically. (Default=don't flip)\n");
	}
	return fResult;
}

int main(int argc, char **argv)
{
	// parse parameters
	if (parse(argc, argv) == false)
		return -1;

	// load fingertip coordinates
	// (if the file does not exist, this call will just fail.)
	// (then the user may measure fingertip coordinates and save them.)
	if (gFingertipPoseEstimation.LoadFingertipCoordinates(gszFingertipFilename) == false)
		printf("fingertip coordinate '%s' file was not loaded.\n", gszFingertipFilename);
	else
		printf("fingertip coordinate '%s' file was loaded.\n", gszFingertipFilename);

	// initialize capture
	if (fInputVideoFile) {
		// Capture From File
		if (!gCapture.Initialize(fFlipFrame, -1, gszInputVideoFilename)) {
			fprintf(stderr, "capture initialization failed.\n");
			return -1;
		}
	} else {
		// Live Capture From Camera
		if (!gCapture.Initialize(fFlipFrame , -1)) {
			fprintf(stderr, "capture initialization failed.\n");
			return -1;
		}
	}

	// OpenGL loop
	glutInit(&argc, &argv[0]);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(100, 100);

	// Init Capture
	IplImage * frame = 0;
	gCapture.CaptureFrame();
	frame = gCapture.QueryFrame();
	if (!frame) {
		fprintf(stderr, "failed to capture a frame...\n");
		return -1;
	}

	// Create Glut Window
	glutInitWindowSize(frame->width, frame->height);
	glutCreateWindow("Handy AR Demo Application");

	// Init OpenGL and Glut
	init();

	// Init Fingertip Pose Estimation
	if (gFingertipPoseEstimation.Initialize(frame, (char *)CALIBRATION_FILENAME) == false)
		return -1;

	// load sequence files
	if (gnModel == MODEL_MAGIC) {
		GLMetaseqInitialize();
		mqo_seq = mqoCreateSequence((char *)SEQ_NAME, N_FRAME, 1.0);
		if (mqo_seq.n_frame <= 0)
		{
			fprintf(stderr, "failed to load sequence files...\n");
			return -1;
		}
	}

	// initialize video writer
	gpRecord = cvCreateVideoWriter(RECORD_FILENAME, -1, 15, cvGetSize(frame));
	gpRecordOutput = cvCreateVideoWriter(RECORD_FILENAME_OUTPUT, -1, 15, cvGetSize(frame));

	// Start main Glut loop
	glutMainLoop();

	return 0;
}
