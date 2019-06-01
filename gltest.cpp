//#include <windows.h>


#include <GL/glut.h>  
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define MAX_JOINT_DATA	16
#define MAX_MOTION		15
#define MAX_JOINT		9

float angle[16], position[18][3], position2[18][3], cam_position[8][3], cam_position2[8][3];
float camera_pos(int leg, float ang[17], float pos[10][3]);
int camera_pos_b(float ang[16], float pos[11][3]);
void camera_area(float wide_h, float wide_v, float cam_pos[4][3]);
int calcPositionOnFloor(float x, float y, float *rx, float *ry);
int calcCamPosition(float rx, float ry, float *cx, float *cy);
float cam_ang_h,cam_ang_v,cam_len;
int mx0, my0;
int play_no;
extern float the;	// temp

int motion_data[MAX_MOTION][MAX_JOINT_DATA] = {
	{-1,21,-42,28,-2,-8,-3,41,-77,43,2,8,0,0,0,45},
	{0,21,-45,28,-1,-12,-1,25,-53,32,1,12,0,0,0,45},
	{1,40,-77,43,-1,-7,0,22,-45,30,2,7,1,0,0,45},
	{0,36,-68,39,-1,0,0,24,-44,28,1,0,-1,0,0,45},
	{0,26,-50,31,-1,-1,-1,35,-65,38,1,1,-1,0,0,45},
	{-1,22,-42,28,-2,-8,-3,43,-77,43,1,8,0,0,0,45},
	{0,22,-45,28,-1,-12,-1,26,-53,32,1,12,0,0,0,45},
	{1,40,-77,43,-1,-7,-1,22,-45,30,2,7,1,0,0,45},
	{0,36,-68,39,-1,0,-1,23,-44,28,1,0,-1,0,0,45},
	{-1,26,-50,31,-1,-1,-2,35,-65,38,1,1,-1,0,0,45},
	{0,22,-42,28,-2,-8,-2,42,-77,43,1,8,0,0,0,45},
	{0,22,-45,28,-1,-12,-1,26,-53,32,1,12,0,0,0,45},
	{1,40,-77,43,-1,-7,0,22,-45,30,2,7,0,0,0,45},
	{0,36,-68,36,-1,0,0,23,-44,25,1,0,-1,0,0,45},
	{0,25,-51,27,-1,0,0,23,-45,24,1,0,0,0,0,45}
};

/*
int corr[MAX_JOINT][2][2] = {
	{{ 2,15},{ 2,15}},
	{{ 3,14},{ 3,14}},
	{{ 4,12},{11,12}},
	{{ 5, 5},{12,11}},
	{{ 6, 4},{13,10}},
	{{ 7, 3},{14, 9}},
	{{ 8, 2},{15, 8}},
	{{ 9, 1},{16, 7}},
	{{10, 0},{17, 6}}
};
*/

void myinit(void)
{
    int i;

	for(i = 0;i < 16;i ++) angle[i] = 0.0f;
	angle[15] = 3.14159/4.0;
	cam_ang_h = cam_ang_v = 1.0f;
	cam_len = 100.0f;
	play_no = -1;

	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glEnable(GL_DEPTH_TEST);
}

void display(void)                /* CALLBACK ������ */
{
    int i;
	float x,y,cx, cy, rx ,ry;
	static int no[4][2] = {{4,5},{5,6},{6,7},{7,4}};

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(cam_len*cos(cam_ang_h)*cos(cam_ang_v), cam_len*sin(cam_ang_h)*cos(cam_ang_v), cam_len*sin(cam_ang_v), 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
	
	glBegin(GL_LINES);
	glNormal3f(0.0f, 0.0f, 1.0f);
		glVertex3f(-2.0f, 0.0f, 0.0f); glVertex3f( 2.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f,-2.0f, 0.0f); glVertex3f( 0.0f, 2.0f, 0.0f);
		
		glVertex3f(-1.0f, 1.0f, 0.0f); glVertex3f( 1.0f, 1.0f, 0.0f);
		glVertex3f( 1.0f, 1.0f, 0.0f); glVertex3f( 1.0f,-1.0f, 0.0f);
		glVertex3f( 1.0f,-1.0f, 0.0f); glVertex3f(-1.0f,-1.0f, 0.0f);
		glVertex3f(-1.0f,-1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 0.0f);

		glVertex3f(position[0][0]/100, position[0][1]/100, 0.0f);
		for(i = 0;i < 11;i ++){		// �X�P���g��������
			glVertex3f(position[i][0]/100, position[i][1]/100, position[i][2]/100);
			if (i == 10) break;
			glVertex3f(position[i][0]/100, position[i][1]/100, position [i][2]/100);
		}
//		glVertex3f(position2[0][0]/100, position2[0][1]/100, 0.0f);
//		for(i = 0;i < 11;i ++){		// �X�P���g��������
//			glVertex3f(position2[i][0]/100, position2[i][1]/100, position2[i][2]/100);
//			if (i == 10) break;
//			glVertex3f(position2[i][0]/100, position2[i][1]/100, position2[i][2]/100);
//		}
		for(i = 11;i < 18;i ++){		// �X�P���g��������
			glVertex3f(position[i  ][0]/100, position[i  ][1]/100, position[i  ][2]/100);
			if (i == 17) break;
			glVertex3f(position[i+1][0]/100, position[i+1][1]/100, position[i+1][2]/100);
		}
		glVertex3f(position[6][0]/100, position[6][1]/100, position[6][2]/100);
		for(i = 0;i < 4;i ++){		// �J�����̉���������������
			glVertex3f(position[9][0]/100, position[9][1]/100, position[9][2]/100);
			glVertex3f(cam_position[i][0]/100, cam_position[i][1]/100, cam_position[i][2]/100);
//			glVertex3f(position2[9][0]/100, position2[9][1]/100, position2[9][2]/100);
//			glVertex3f(cam_position2[i][0]/100, cam_position2[i][1]/100, cam_position2[i][2]/100);
		}
		for(i = 0;i < 4;i ++){		// �n�ʂ̃J�����̘g������
			if ((cam_position[no[i][0]][2] == 0.0f)&&(cam_position[no[i][1]][2] == 0.0f)){
				glVertex3f(cam_position[no[i][0]][0]/100, cam_position[no[i][0]][1]/100, 0.0f);
				glVertex3f(cam_position[no[i][1]][0]/100, cam_position[no[i][1]][1]/100, 0.0f);
			}
//			if ((cam_position2[no[i][0]][2] == 0.0f)&&(cam_position2[no[i][1]][2] == 0.0f)){
//				glVertex3f(cam_position2[no[i][0]][0]/100, cam_position2[no[i][0]][1]/100, 0.0f);
//				glVertex3f(cam_position2[no[i][1]][0]/100, cam_position2[no[i][1]][1]/100, 0.0f);
//			}
		}
		for(x = -1000;x <= 1000;x += 200){
			for(y = -1000;y <= 1000;y += 200){
				if (!calcCamPosition((float)x, (float)y, &cx, &cy)){
					if ((cx < -0.5f)||(cx > 0.5f)||(cy < -0.5f*0.75f)||(cy > 0.5f*0.75f)) continue;
					if (!calcPositionOnFloor(cx, cy, &rx, &ry)){
						glVertex3f(rx/100+0.1, ry/100+0.1, 0.0f); glVertex3f(rx/100+0.1, ry/100-0.1, 0.0f);
						glVertex3f(rx/100+0.1, ry/100-0.1, 0.0f); glVertex3f(rx/100-0.1, ry/100-0.1, 0.0f);
						glVertex3f(rx/100-0.1, ry/100-0.1, 0.0f); glVertex3f(rx/100-0.1, ry/100+0.1, 0.0f);
						glVertex3f(rx/100-0.1, ry/100+0.1, 0.0f); glVertex3f(rx/100+0.1, ry/100+0.1, 0.0f);
					}
				}
			}
		}
		glEnd();

    glFlush( );
}

float deg2rad(float deg){
	return( deg / 180.0f * 3.14159f );
}

void keyboard(unsigned char key, int x, int y)
{
	int i;
	
	switch (key) {
		case '1': angle[ 0] += 0.1f; break;
		case 'q': angle[ 0] -= 0.1f; break;
		case '2': angle[ 1] += 0.1f; break;
		case 'w': angle[ 1] -= 0.1f; break;
		case '3': angle[ 2] += 0.1f; break;
		case 'e': angle[ 2] -= 0.1f; break;
		case '4': angle[ 3] += 0.1f; break;
		case 'r': angle[ 3] -= 0.1f; break;
		case '5': angle[ 4] += 0.1f; break;
		case 't': angle[ 4] -= 0.1f; break;
		case '6': angle[ 5] += 0.1f; break;
		case 'y': angle[ 5] -= 0.1f; break;
		case '7': angle[ 6] += 0.1f; break;
		case 'u': angle[ 6] -= 0.1f; break;
		case '8': angle[ 7] += 0.1f; break;
		case 'i': angle[ 7] -= 0.1f; break;
		case '9': angle[ 8] += 0.1f; break;
		case 'o': angle[ 8] -= 0.1f; break;
		case '0': angle[ 9] += 0.1f; break;
		case 'p': angle[ 9] -= 0.1f; break;
		case '-': angle[10] += 0.1f; break;
		case '@': angle[10] -= 0.1f; break;
		case '^': angle[11] += 0.1f; break;
		case '[': angle[11] -= 0.1f; break;
		case 'a': angle[12] += 0.1f; break;
		case 'z': angle[12] -= 0.1f; break;
		case 's': angle[14] += 0.1f; break;
		case 'x': angle[14] -= 0.1f; break;
		case 'd': angle[15] += 0.1f; break;
		case 'c': angle[15] -= 0.1f; break;
		case 'f': cam_ang_h -= 0.1f; break;
		case 'v': cam_ang_h += 0.1f; break;
		case 'g': cam_ang_v -= 0.1f; break;
		case 'b': cam_ang_v += 0.1f; break;
		case 'h': cam_len -= 1.0f; break;
		case 'n': cam_len += 1.0f; break;
		case 'j':
		{
			if (play_no < (MAX_MOTION - 1)) play_no ++;
			for(i = 0;i < 16;i ++){
				angle[i] = deg2rad((float)motion_data[play_no][i]);
			}
			break;
		}
		case 'm':
		{
			if (play_no > 0) play_no --;
			for(i = 0;i < 16;i ++){
				angle[i] = deg2rad((float)motion_data[play_no][i]);
			}
			break;
		}

		case '\033': exit(0);
		default: break;
	}
	camera_pos_b(angle, position);			// �n�ʂɋ߂��Ɨ\������鑫������̃J�����̈ʒu�Ǝp��
	camera_area(0.5f,0.5f*0.75f,cam_position);
	glutPostRedisplay();
}

void mouse(int button , int state , int x , int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
		mx0 = x; my0 = y;
	}
}

void motion(int x , int y) {
	cam_ang_h -= (float)(x - mx0)*0.01;
	cam_ang_v += (float)(y - my0)*0.01;
	mx0 = x; my0 = y;
	glutPostRedisplay();
}

void myReshape(int w, int h) /* CALLBACK ������ */
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(10.0, (GLfloat)w/(GLfloat)h, 1.0, 10000.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(500, 500);
    glutCreateWindow("test");
    myinit();
    glutReshapeFunc(myReshape);
    glutDisplayFunc(display);
	
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);

    glutMainLoop( );
    return 0;
}
