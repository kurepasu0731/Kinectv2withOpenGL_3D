//�\���}�l�[�W���̃A�N�e�B�u�\�����[�V�����v���b�g�t�H�[����x64�ŁI(freeglut��dll��lib��x64�p�Ƀr���h��������)
#include <gl/freeglut.h>
#include <stdio.h>
#include <Kinect.h>
#include <iostream>

#define ERROR_CHECK2(ret, name) \
	if(FAILED(ret)){ \
		printf("failed at %s\n", name); \
	} \

#define ERROR_CHECK(ret) \
	if(FAILED(ret)){ \
		printf("failed at %x\n", ret); \
	} \

//Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if(pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

IKinectSensor* kinect = NULL;
IMultiSourceFrameReader *multiFrameReader;
int colorWidth;
int colorHeight;
ColorImageFormat imageFormat;
float (*colorMap)[3] = NULL;
BYTE *bodyIndexBuffer = NULL;
int width = 512;
int height = 424;
ICoordinateMapper* coordinateMapper;
ColorSpacePoint *colorCoordinates = NULL;
CameraSpacePoint *cameraSpacePoints = NULL;
float theta = 135; //135
float fai = 0;
float r = 1.0f;

//�J���[�}�b�v�p�̃�����
RGBQUAD *colorRGBX;

void kinectInit() {
  ERROR_CHECK(GetDefaultKinectSensor(&kinect));
  ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));
  ERROR_CHECK(kinect->Open());
  ERROR_CHECK(kinect->OpenMultiSourceFrameReader(
			 FrameSourceTypes::FrameSourceTypes_Depth
		       | FrameSourceTypes::FrameSourceTypes_Color
			   | FrameSourceTypes::FrameSourceTypes_BodyIndex,
		       &multiFrameReader));

}

void capture()
{
	IMultiSourceFrame *multiFrame = NULL;

	IColorFrame *colorFrame = NULL;
	IColorFrameReference *colorFrameReference = NULL;
	UINT colorBufferSize = 0;
	RGBQUAD *colorBuffer = NULL;

	IDepthFrame *depthFrame = NULL;
	IDepthFrameReference *depthFrameReference = NULL;
	UINT bufferSize = 0;
	UINT16 *depthBuffer = NULL;

	IBodyIndexFrame *bodyIndexFrame = NULL;
	IBodyIndexFrameReference *bodyIndexFrameReference = NULL;
	UINT bodyIndexBufferSize = 0;
	static int lastTime = 0;
	static int currentTime = 0;
	HRESULT hr = -1;

	//�t���[�����[�_�[���ǂݍ��݉\�ɂȂ�̂�҂��[�v(�eFrame�������Ȃ�)
  while(1) {
	  if((currentTime = GetTickCount()) > 33)
	  {
	    hr = multiFrameReader->AcquireLatestFrame(&multiFrame);
		lastTime = currentTime;
	  }else continue;

    if(FAILED(hr)) {
       //fprintf(stderr, "AcquireLatestFrame(&multiFrame)\n");
      Sleep(1);
      continue;
    }
    
    hr = multiFrame->get_ColorFrameReference(&colorFrameReference);
    if(FAILED(hr)) {
      Sleep(1);
      fprintf(stderr, "ColorFrameReference(&colorFrameReference)\n");
      SafeRelease(multiFrame);
      continue;
    }

    hr = colorFrameReference->AcquireFrame(&colorFrame);
    if(FAILED(hr)) {
      Sleep(1);
      fprintf(stderr, "AcquireFrame(&colorFrame)\n");		
      SafeRelease(colorFrameReference);
      SafeRelease(multiFrame);
      continue;
    }

	hr = multiFrame->get_DepthFrameReference(&depthFrameReference);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "DepthFrameReference(&depthFrameReference)\n");
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}

	hr = depthFrameReference->AcquireFrame(&depthFrame);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "AcquireFrame(&depthFrame)\n");
		SafeRelease(depthFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}

	hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &depthBuffer);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "AccessUnderlyingBuffer(&bufferSize, &depthBuffer\n");
		SafeRelease(depthFrame);
		SafeRelease(depthFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}


	hr = multiFrame->get_BodyIndexFrameReference(&bodyIndexFrameReference);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "BodyIndexReference(&colorFrameReference)\n");
		free(depthBuffer);
		SafeRelease(depthFrame);
		SafeRelease(depthFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}

	hr = bodyIndexFrameReference->AcquireFrame(&bodyIndexFrame);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "AcquireFrame(&bodyIndexFrame)\n");
		SafeRelease(bodyIndexFrameReference);
		free(depthBuffer);
		SafeRelease(depthFrame);
		SafeRelease(depthFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}

	hr = bodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexBufferSize, &bodyIndexBuffer);
	if(FAILED(hr))
	{
		Sleep(1);
		fprintf(stderr, "bodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexBufferSize, &bodyIndexBuffer)");
		SafeRelease(bodyIndexFrame);
		SafeRelease(bodyIndexFrameReference);
		free(depthBuffer);
		SafeRelease(depthFrame);
		SafeRelease(depthFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(colorFrameReference);
		SafeRelease(multiFrame);
		continue;
	}

/*
	hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &depthBuffer);
	if (FAILED(hr)) {
		Sleep(1);
		fprintf(stderr, "AccessUnderlyingBuffer(&bufferSize, &depthBuffer\n");
		SafeRelease(depthFrame);
		SafeRelease(depthFrameReference);
		SafeRelease(bodyIndexFrame);
		SafeRelease(bodyIndexFrameReference);
		SafeRelease(colorFrameReference);
		SafeRelease(colorFrame);
		SafeRelease(multiFrame);
		continue;
	}
*/
    SafeRelease(colorFrameReference);
	SafeRelease(bodyIndexFrameReference);
	SafeRelease(depthFrameReference);
    break;
  }

  //�[�x�l��Buffer�֊i�[
 // ERROR_CHECK(depthFrame->AccessUnderlyingBuffer(&bufferSize, &depthBuffer));

	//�J���[�}�b�v�̐ݒ�f�[�^�̓ǂݍ��݂ƁAcolorBuffer�������̊m��
	if(colorRGBX == NULL)
	{
		IFrameDescription *colorFrameDescription = NULL;

		ERROR_CHECK2(colorFrame->get_FrameDescription(&colorFrameDescription), "FrameDescription");
		ERROR_CHECK2(colorFrameDescription->get_Width(&colorWidth), "get_Width");
		ERROR_CHECK2(colorFrameDescription->get_Height(&colorHeight), "get_Height");
		colorRGBX = new RGBQUAD[colorWidth * colorHeight];

		glutReshapeWindow(width, height);

		ERROR_CHECK2(colorFrame->get_RawColorImageFormat(&imageFormat), "get_RawColorImageFormat");

		SafeRelease(colorFrameDescription);
	}

	//�J���[�C���[�W��colorBuffer�փR�s�[
	if(imageFormat == ColorImageFormat_Bgra)
	{
		ERROR_CHECK2(colorFrame->AccessRawUnderlyingBuffer(&colorBufferSize,
			reinterpret_cast<BYTE**>(&colorBuffer)), "AccessRawUnderlyingBuffer");
	}else if(colorRGBX)
	{
		colorBuffer = colorRGBX;
		colorBufferSize = colorWidth * colorHeight * sizeof(RGBQUAD);
		ERROR_CHECK2(colorFrame->CopyConvertedFrameDataToArray(colorBufferSize,
			reinterpret_cast<BYTE*>(colorBuffer),
			ColorImageFormat_Bgra), "CopyConvertedFrameDataToArray");
	}else
	{
		//Error
	}

	//colorMap�̏������@��x�������s�����
	if(colorMap == NULL)
	{
		colorMap = new float[colorWidth * colorHeight][3];
	}
	if (colorCoordinates == NULL)
	{
		colorCoordinates = new ColorSpacePoint[width * height];
	}
	if (cameraSpacePoints == NULL)
	{
		cameraSpacePoints = new CameraSpacePoint[width * height];
	}

	ERROR_CHECK2(coordinateMapper->MapDepthFrameToColorSpace(
	      width * height, (UINT16*)depthBuffer, 
              width * height, colorCoordinates), "MapDepthFrameToColorSpace");

	ERROR_CHECK2(coordinateMapper->MapDepthFrameToCameraSpace(
	      width * height, (UINT16*)depthBuffer, 
	      width * height, cameraSpacePoints),"MapDepthFrameToCameraSpace");

	//colorBuffer�̃f�[�^���AcolorMap�ɃR�s�[
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			int index = i * width + j;

			ColorSpacePoint colorPoint = colorCoordinates[index];
			int colorX = (int)(floor(colorPoint.X + 0.5));
			int colorY = (int)(floor(colorPoint.Y + 0.5));

			if(colorX >= 0 && colorX < colorWidth && colorY >= 0 && colorY < colorHeight)
			{
				int colorIndex = colorX + colorY * colorWidth;
				//�i�[��̓~���[���[�h������
				float* colorp = colorMap[index];

				UCHAR* data = (UCHAR*)(colorBuffer + colorIndex);
				colorp[0] = (float)data[2] / 255.0f;
				colorp[1] = (float)data[1] / 255.0f;
				colorp[2] = (float)data[0] / 255.0f;
			}else
			{
				float* colorp = colorMap[index];
				colorp[0] = 0;
				colorp[1] = 0;
				colorp[2] = 0;
			}
		}
	}

  //static UINT bodyIndexBufferSize = 512 * 424;
 // bodyIndexBuffer = new BYTE[bodyIndexBufferSize];
 // ERROR_CHECK(bodyIndexFrame->CopyFrameDataToArray(bodyIndexBufferSize, &bodyIndexBuffer[0]));

	//�t���[�����\�[�X�����
  SafeRelease(colorFrame);
  SafeRelease(multiFrame);
  SafeRelease(bodyIndexFrame);
  SafeRelease(depthFrame);
}

void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	  case 'j':
		theta = theta - 0.1f;
		break;
	  case 'k':
		theta = theta + 0.1f;
		break;
	  case 'h':
		fai = fai + 0.1f;
		break;
	  case 'l':
		fai = fai - 0.1f;
		break;
	  case 'i':
		  r = r * 1.1f;
		  break;
	  case 'm':
		  r = r * 0.9f;
		  break;
	  case 'q':
		kinect->Close();
		kinect->Release();
		exit(0);
		break;
	}
}

void display()
{
	//��ʂ̃L���v�`��
	capture();

	glEnable(GL_DEPTH_TEST);

	//��ʂ��ׂ����̂ŁA�K���ȃs�b�`�ɑe������
	int pitch = 1;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); //�ϊ��s��̏�����
	float cx = 0.0f;
	float cy = 0.0f;
	float cz = 1.0f;
	float x = r * cos(fai) * sin(theta) + cx;
	float y = r * sin(fai) + cy;
	float z = r * cos(fai) * cos(theta) + cz;
	gluLookAt(x, y, z, cx, cy, cz, 0.0f, 1.0f, 0.0f);

	//height - 1, width - 1�܂�
	for(int i = 0; i < height - pitch; i += pitch){
		for(int j= 0; j < width - pitch; j+= pitch){
			int index = i * width + j;
			float z0 = cameraSpacePoints[index].Z;
			float z1 = cameraSpacePoints[index + pitch].Z;
			float z2 = cameraSpacePoints[index + width*pitch + pitch].Z;
			float z3 = cameraSpacePoints[index + width*pitch].Z;
			if(z0 > 0 && z1 > 0 && z2 > 0 && z3 > 0)
			{
				float x0 = cameraSpacePoints[index].X;
				float x1 = cameraSpacePoints[index + pitch].X;
				float x2 = cameraSpacePoints[index + width*pitch + pitch].X;
				float x3 = cameraSpacePoints[index + width*pitch].X;
				float y0 = cameraSpacePoints[index].Y;
				float y1 = cameraSpacePoints[index + pitch].Y;
				float y2 = cameraSpacePoints[index + width*pitch + pitch].Y;
				float y3 = cameraSpacePoints[index + width*pitch].Y;
				glBegin(GL_POLYGON);
				glColor3f(colorMap[index][0], colorMap[index][1], colorMap[index][2]);
				glVertex3f(x0, y0, z0);
				glVertex3f(x1, y1, z1);
				glVertex3f(x2, y2, z2);
				glVertex3f(x3, y3, z3);
				glEnd();
			}
		}
	}
	glutSwapBuffers();
}

//idle��ԂɂȂ�����A�ĕ`�����������
//����ɂ��A�������[�v�̕\�����\�ɂȂ�
void idle()
{
	glutPostRedisplay();
}

int main(int argc, char **argv){
	printf("size of RGBQUAD %d\n", sizeof(RGBQUAD));
	printf("size of CameraSpacePoint %d\n", sizeof(CameraSpacePoint));
	//Kinect�̏�����
	kinectInit();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); //�_�u���o�b�t�@�A�f�u�X�o�b�t�@�p��
	glEnable(GL_DEPTH_TEST);

	glutCreateWindow("Kinect");

	glMatrixMode(GL_PROJECTION); //�������e�ϊ�
	glLoadIdentity(); //�ϊ��s�񏉊���
	gluPerspective(45.0f, (float)width/height, 0.5f, 4.0f); //�N���b�s���O�̈�w��

	glMatrixMode(GL_MODELVIEW);
	//�n���h���֐��̐ݒ�
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();
}
