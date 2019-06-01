//#include <windows.h>
#include <stdio.h>
#include <math.h>

#define SUPPORT_LEG_RIGHT	0
#define SUPPORT_LEG_LEFT	1

enum joint{
	CAM,H,HE,H1,BR1,LR1,LR2,LR3,LR4,LR5,LR6,BL1,LL1,LL2,LL3,LL4,LL5,LL6
};

int corr[18] = { -1, -1,15,14,12, 5, 4, 3, 2, 1, 0,12,11,10, 9, 8, 7, 6 };

struct link_para_T{
	float p[3];
	float a[3];
};

struct link_para_T link_para[18] = 
{
	{{300.0f,   0.0f,   0.0f},{  0.0f,  0.0f,  0.0f}},	// CAM
	{{ 41.5f,  20.6f,  66.1f},{  0.0f,  0.0f,  1.0f}},	// H
	{{ 28.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// HE
	{{  0.0f,   0.0f, 166.0f},{  0.0f,  0.0f,  1.0f}},	// H1

	{{  0.0f,  44.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// BR1
	{{  0.0f,   0.0f,  45.8f},{  0.0f,  0.0f,  1.0f}},	// LR1
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// LR2
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LR3
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LR4
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LR5
	{{  0.0f,   0.0f,  66.0f},{  1.0f,  0.0f,  0.0f}},	// LR6

	{{  0.0f, -44.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// BL1
	{{  0.0f,   0.0f,  45.8f},{  0.0f,  0.0f,  1.0f}},	// LL1
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// LL2
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LL3
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LL4
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LL5
	{{  0.0f,   0.0f,  66.0f},{  1.0f,  0.0f,  0.0f}}	// LL6
};

struct link_para_T link_para2[18] = 
{
	{{300.0f,   0.0f,   0.0f},{  0.0f,  0.0f,  0.0f}},	// CAM
	{{ 41.5f,  20.6f,  66.1f},{  0.0f,  0.0f,  1.0f}},	// H
	{{ 28.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// HE
	{{  0.0f,   0.0f, 166.0f},{  0.0f,  0.0f,  1.0f}},	// H1

	{{  0.0f,  44.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// BR1
	{{  0.0f,   0.0f,  45.8f},{  0.0f,  0.0f,  1.0f}},	// LR1
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// LR2
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LR3
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LR4
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LR5
	{{  0.0f,   0.0f,  66.0f},{  1.0f,  0.0f,  0.0f}},	// LR6

	{{  0.0f, -44.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// BL1
	{{  0.0f,   0.0f,  45.8f},{  0.0f,  0.0f,  1.0f}},	// LL1
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// LL2
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LL3
	{{  0.0f,   0.0f, 118.0f},{  0.0f,  1.0f,  0.0f}},	// LL4
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LL5
	{{  0.0f,   0.0f,  66.0f},{  1.0f,  0.0f,  0.0f}}	// LL6
};

float cam_matrix[4][4],cam_matrix_t[2][4][4];

float *linkMatrix(int no, float ang, float matrix[4][4]){
	float c = (float)cos(ang), s = (float)sin(ang);
	
	matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f; matrix[0][3] = link_para[no].p[0];
	matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f; matrix[1][3] = link_para[no].p[1];
	matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f; matrix[2][3] = link_para[no].p[2];
	matrix[3][0] = 0.0f; matrix[3][1] = 0.0f; matrix[3][2] = 0.0f; matrix[3][3] = 1.0f;
	for(int i = 0;i < 3;i ++){
		if (link_para[no].a[i] != 0){
			matrix[(i+1)%3][(i+1)%3] =  c * link_para[no].a[i];
			matrix[(i+1)%3][(i+2)%3] = -s * link_para[no].a[i];
			matrix[(i+2)%3][(i+1)%3] =  s * link_para[no].a[i];
			matrix[(i+2)%3][(i+2)%3] =  c * link_para[no].a[i];
		}
	}
	return matrix[0];
}

float *linkMatrix2(int no, float ang, float matrix[4][4]){
	float c = (float)cos(ang), s = (float)sin(ang);
	
	matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f; matrix[0][3] = link_para2[no].p[0];
	matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f; matrix[1][3] = link_para2[no].p[1];
	matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f; matrix[2][3] = link_para2[no].p[2];
	matrix[3][0] = 0.0f; matrix[3][1] = 0.0f; matrix[3][2] = 0.0f; matrix[3][3] = 1.0f;
	for(int i = 0;i < 3;i ++){
		if (link_para2[no].a[i] != 0){
			matrix[(i+1)%3][(i+1)%3] =  c * link_para2[no].a[i];
			matrix[(i+1)%3][(i+2)%3] = -s * link_para2[no].a[i];
			matrix[(i+2)%3][(i+1)%3] =  s * link_para2[no].a[i];
			matrix[(i+2)%3][(i+2)%3] =  c * link_para2[no].a[i];
		}
	}
	return matrix[0];
}

float *mulMatrix(float a[4][4], float b[4][4], float ans[4][4]){
	for(int i = 0;i < 4;i ++){
		for(int j = 0;j < 4;j ++){
			ans[i][j] = 0.0f;
			for(int k = 0;k < 4;k ++){
				ans[i][j] += a[i][k] * b[k][j];
			}
		}
	}
	return ans[0];
}

int invMatrix(float a[3][3], float ans[3][3]){
	float mat,sign;
	
	mat = a[0][0]*a[1][1]*a[2][2] + a[0][1]*a[1][2]*a[2][0] + a[0][2]*a[1][0]*a[2][1] -
		a[0][2]*a[1][1]*a[2][0] - a[0][1]*a[1][0]*a[2][2] - a[0][0]*a[1][2]*a[2][1];
	if (mat == 0.0f) return -1;
	for(int i = 0;i < 3;i ++){
		for(int j = 0;j < 3;j ++){
			sign = 1.0f;
			ans[j][i] = sign*(a[(i+1)%3][(j+1)%3]*a[(i+2)%3][(j+2)%3]-a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3])/mat;
		}
	}
	return 0;
}

float *copyMatrix(float d[4][4], float s[4][4]){
	for(int i = 0;i < 4;i ++)
		for(int j = 0;j < 4;j ++)
			d[i][j] = s[i][j];
	return d[0];
}


float camera_pos(int leg, float ang_t[16], float pos[11][3]){
	float link[18][4][4];
	float kine[11][4][4];
	float ang[18];

	for(int i = 0;i < 18;i ++){
		if (corr[i] >= 0){
			ang[i] = ang_t[corr[i]];
		} else {
			ang[i] = 0;
		}
	}
	
	for(int i = 0;i < 18;i ++){
		linkMatrix(i,ang[i],link[i]);
	}

	if (leg == SUPPORT_LEG_RIGHT){
		copyMatrix(kine[0],link[LR6]);
		for(int i = LR5;i >= BR1;i --){
			mulMatrix(kine[LR5-i],link[i],kine[LR5-i+1]);
		}
	} else if (leg == SUPPORT_LEG_LEFT){
		copyMatrix(kine[0],link[LL6]);
		for(int i = LL5;i >= BL1;i --){
			mulMatrix(kine[LL5-i],link[i],kine[LL5-i+1]);
		}
	}
	mulMatrix(kine[6],link[H1],kine[7]);
	mulMatrix(kine[7],link[HE],kine[8]);
	mulMatrix(kine[8],link[H],kine[9]);
	mulMatrix(kine[9],link[CAM],kine[10]);
	copyMatrix(cam_matrix,kine[9]);

	for(int i = 0;i < 11;i ++){
		pos[i][0] = kine[i][0][3];
		pos[i][1] = kine[i][1][3];
		pos[i][2] = kine[i][2][3];
	}

	return 0;
}

float the = 0.0f;

float camera_pos_from_waist(int *leg, float ang_t[16], float pos[11][3]){
	float link[18][4][4], right[7][4][4], left[7][4][4];
	float kine[11][4][4], cam[4][4], rot[4][4];
	float ang[18], x0[3], x1[3], s, c, the, the2;

	for(int i = 0;i < 18;i ++){					// 関節角をセット
		if (corr[i] >= 0){
			ang[i] = ang_t[corr[i]];
		} else {
			ang[i] = 0;
		}
	}
	for(int i = 0;i < 18;i ++){					// リンクマトリクス
		linkMatrix(i,ang[i],link[i]);
	}
	copyMatrix(right[0],link[LR6]);			// 右足の計算
	for(int i = LR5;i >= BR1;i --){
		mulMatrix(right[LR5-i],link[i],right[LR5-i+1]);
	}
	copyMatrix(left[0],link[LL6]);			// 左足の計算
	for(int i = LL5;i >= BL1;i --){
		mulMatrix(left[LL5-i],link[i],left[LL5-i+1]);
	}
	if (right[6][2][3] > left[6][2][3]){
		for(int i = 0;i < 7;i ++) copyMatrix(kine[i],right[i]);
		*leg = SUPPORT_LEG_RIGHT;
	} else {
		for(int i = 0;i < 7;i ++) copyMatrix(kine[i],left[i]);
		*leg = SUPPORT_LEG_LEFT;
	}
	mulMatrix(kine[6],link[H1],kine[7]);		// 腰→首
	mulMatrix(kine[7],link[HE],kine[8]);		// 首旋回
	mulMatrix(kine[8],link[H],kine[9]);			// 首上下
	mulMatrix(kine[9],link[CAM],kine[10]);		// カメラ光軸

	x0[0] = kine[6][0][3];						// 腰の位置を計算する
	x0[1] = kine[6][1][3];
	x0[2] = 0.0f;
	the = atan2(kine[6][1][0],kine[6][0][0]);	// 足先に対する腰の角度を計算する

	s = sin(the); c = cos(the);
	// 腰中心に替えるための回転行列を求める
	rot[0][0] =  c, rot[0][1] = s; rot[0][2] = 0.0f; rot[0][3] = -x0[0];
	rot[1][0] = -s, rot[1][1] = c; rot[1][2] = 0.0f; rot[1][3] = -x0[1];
	rot[2][0] = 0.0f, rot[2][1] = 0.0f; rot[2][2] = 1.0f; rot[2][3] = 0.0f;
	rot[3][0] = 0.0f, rot[3][1] = 0.0f; rot[3][2] = 0.0f; rot[3][3] = 1.0f;
	
	copyMatrix(cam, kine[9]);					// 足先を基準とするカメラの位置を計算する
	mulMatrix(rot, cam, cam_matrix);

	// 各関節の位置を腰中心に移動
	for(int i = 0;i < 11;i ++){
		pos[i][0] =  (kine[i][0][3] - x0[0])*c+(kine[i][1][3] - x0[1])*s;
		pos[i][1] = -(kine[i][0][3] - x0[0])*s+(kine[i][1][3] - x0[1])*c;
		pos[i][2] = kine[i][2][3] - x0[2];
	}

	if (*leg  == SUPPORT_LEG_RIGHT){
		x1[0] = left[6][0][3];
		x1[1] = left[6][1][3];
		x1[2] = left[6][2][3] - right[6][2][3];
		the2 = atan2(left[6][1][0],left[6][0][0]) - the;	// 足先に対する腰の角度を計算する
		// 各関節の位置を腰中心に移動
		s = sin(the2); c = cos(the2);
		pos[11][0] = -x1[0] * c - x1[1] * s;
		pos[11][1] =  x1[0] * s - x1[1] * c;
		pos[11][2] = -x1[2];
		for(int i = 0;i < 6;i ++){
			pos[i+12][0] =  (left[i][0][3] - x1[0]) * c + (left[i][1][3] - x1[1]) * s;
			pos[i+12][1] = -(left[i][0][3] - x1[0]) * s + (left[i][1][3] - x1[1]) * c;
			pos[i+12][2] =   left[i][2][3] - x1[2];
		}
	} else {
		x1[0] = right[6][0][3];
		x1[1] = right[6][1][3];
		x1[2] = right[6][2][3] - left[6][2][3];
		the2 = atan2(right[6][1][0],right[6][0][0]) - the;	// 足先に対する腰の角度を計算する
		s = sin(the2); c = cos(the2);
		pos[11][0] = -x1[0] * c - x1[1] * s;
		pos[11][1] =  x1[0] * s - x1[1] * c;
		pos[11][2] = -x1[2];
		for(int i = 0;i < 6;i ++){
			pos[i+12][0] =  (right[i][0][3] - x1[0]) * c + (right[i][1][3] - x1[1]) * s;
			pos[i+12][1] = -(right[i][0][3] - x1[0]) * s + (right[i][1][3] - x1[1]) * c;
			pos[i+12][2] =   right[i][2][3] - x1[2];
		}
	}

	return 0;
}

// 角度が足先中心になっていることを確認．修正
int camera_pos_b(float ang[16], float pos[11][3]){
	int leg;
	camera_pos_from_waist(&leg,ang,pos);
																	// 右足裏を原点とするカメラの位置を計算
	return leg;
}

void camera_area(float wide_h, float wide_v, float cam_pos[8][3]){
	float vec[3];
	static float sign[4][2] = {{-1.0f,-1.0f},{-1.0f,1.0f},{1.0f,1.0f},{1.0f,-1.0f}};
	float x0 = cam_matrix[0][3], y0 = cam_matrix[1][3], z0 = cam_matrix[2][3];

	for(int i = 0;i < 4;i ++){
		vec[0] = cam_matrix[0][0]+sign[i][0]*cam_matrix[0][1]*wide_h+sign[i][1]*cam_matrix[0][2]*wide_v;
		vec[1] = cam_matrix[1][0]+sign[i][0]*cam_matrix[1][1]*wide_h+sign[i][1]*cam_matrix[1][2]*wide_v;
		vec[2] = cam_matrix[2][0]+sign[i][0]*cam_matrix[2][1]*wide_h+sign[i][1]*cam_matrix[2][2]*wide_v;
		cam_pos[i][0] = 100.0f*vec[0] + x0;
		cam_pos[i][1] = 100.0f*vec[1] + y0;
		cam_pos[i][2] = 100.0f*vec[2] + z0;

		if (vec[2] > 0.0f) cam_pos[i+4][2] = 1.0f;
		else {
			cam_pos[i+4][0] = vec[0]/vec[2]*(0.0f-z0)+x0;
			cam_pos[i+4][1] = vec[1]/vec[2]*(0.0f-z0)+y0;
			cam_pos[i+4][2] = 0.0f;
		}
	}
}

// 画面上の位置をフロアーの位置に変換する
// x,y - カメラから1m先での(x,y)の実際の位置
// 必ずcam_matrixを求めてから呼び出す
int calcPositionOnFloor(float x, float y, float *rx, float *ry){
	float x0 = cam_matrix[0][3], y0 = cam_matrix[1][3], z0 = cam_matrix[2][3];
	float vec[3];
	
	vec[0] = cam_matrix[0][0]+cam_matrix[0][1]*x+cam_matrix[0][2]*y;
	vec[1] = cam_matrix[1][0]+cam_matrix[1][1]*x+cam_matrix[1][2]*y;
	vec[2] = cam_matrix[2][0]+cam_matrix[2][1]*x+cam_matrix[2][2]*y;
	if (vec[2] > 0.0f) return -1;	// 地上には無い（水平線以上にある浮いた物体）
	else {
		*rx = vec[0]/vec[2]*(0.0f-z0)+x0;
		*ry = vec[1]/vec[2]*(0.0f-z0)+y0;
	}
	return 0;
}

int calcCamPosition(float rx, float ry, float *cx, float *cy){
	float x, y, z;
	float x0 = cam_matrix[0][3], y0 = cam_matrix[1][3], z0 = cam_matrix[2][3];
	float vec[3], a[3][3], mat[3][3];
	
	vec[0] = (rx - x0)/(0.0f - z0);			// カメラから地上へのベクトル
	vec[1] = (ry - y0)/(0.0f - z0);
	vec[2] = 1.0f;

	for(int i = 0;i < 3;i ++){
		for(int j = 0;j < 3;j ++){
			a[i][j] = cam_matrix[i][j];
		}
	}
	invMatrix(a,mat);

	z = mat[0][0]*vec[0]+mat[0][1]*vec[1]+mat[0][2]*vec[2];
	x = mat[1][0]*vec[0]+mat[1][1]*vec[1]+mat[1][2]*vec[2];
	y = mat[2][0]*vec[0]+mat[2][1]*vec[1]+mat[2][2]*vec[2];

	if (z == 0.0f) return -1;
	*cx = x/z;
	*cy = y/z;
	return 0;
}

/*
main(){
	int i;
	float ang[17],pos[10][3];

	for(i = 0;i < 17;i ++) ang[i] = 0.0f;
	camera_pos(0,ang,pos);
}
*/

