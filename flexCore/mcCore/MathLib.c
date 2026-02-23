/*****************************************************************//**
 * \file   MathLib.c
 * \brief  自定义的数学函数实现
 * 
 * \author LiBing
 * \date   June 2020
 *********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "MathLib.h"


int VerSion=100;
#define FREE_ARG	char*
#define ZERO_TOL	1.0e-8

/**
 * \brief 计算两个坐标的欧拉距离.
 * 
 * \param n	坐标的维数
 * \param pos1	第一个坐标
 * \param pos2	第二个坐标
 * \return 
 */
float_def euler_dist(int n, float_def* pos1, float_def* pos2)
{
	int i;
	float_def dist = 0.0;
	for (i = 0; i < n; i++)
	{
		dist = dist + (pos2[i] - pos1[i]) * (pos2[i] - pos1[i]);
	}
	return sqrt(dist);
}


/**
 * \brief 计算n维向量的2范数。
 *
 * \param n		向量元素的个数。
 * \param vec	向量数组。
 * \return		返回向量的2范数。
 */
float_def vec_norm(int n, float_def* vec)
{
	int row;
	float_def len = 0.0;
	for (row = 0; row < n; row++)
		len = len + vec[row] * vec[row];
	return sqrt(len);
}

/**
 * \brief		创建一个float_def类型的n维向量，即给n维向量分配内存。
 *
 * \param n		向量的维数。
 * \return		一维数组的指针。
 */
float_def* dvector(int n)
{
	float_def* m;
	m = (float_def*)malloc((size_t)(n) * sizeof(float_def));
	return m;
}


/**
 * \brief		3维向量复制。
 * 
 * \param vec1	3维向量。
 * \param vec2	3维向量。
 */
void vec3_cpy(float_def vec1[3], float_def vec2[3])
{
	vec2[0] = vec1[0];
	vec2[1] = vec1[1];
	vec2[2] = vec1[2];
	return;
}

/**
 * \brief		3维向量相加。
 *
 * \param vec1	3维向量。
 * \param vec2	3维向量。
 * \param vec3	3维向量。
 */
void vec3_add(float_def vec1[3], float_def vec2[3], float_def vec3[3])
{
	vec3[0] = vec1[0] + vec2[0];
	vec3[1] = vec1[1] + vec2[1];
	vec3[2] = vec1[2] + vec2[2];
	return;
}

/**
 * \brief		3维向量相减。
 *
 * \param vec1	3维向量。
 * \param vec2	3维向量。
 * \param vec3	3维向量。
 */
void vec3_sub(float_def vec1[3], float_def vec2[3], float_def vec3[3])
{
	vec3[0] = vec1[0] - vec2[0];
	vec3[1] = vec1[1] - vec2[1];
	vec3[2] = vec1[2] - vec2[2];
	return;
}

/**
 * \brief 任意维向量点乘.
 *
 * \param dim   维数。
 * \param vec1	向量1。
 * \param vec2  向量2。
 * \return		点乘结果。
 */
float_def vec_dot(int dim,float_def *vec1, float_def *vec2)
{
	int i = 0;
	float_def tmp = 0.0;
	for (i = 0; i < dim; i++)
		tmp = tmp + vec1[i] * vec2[i];
	return tmp;
}

/**
 * \brief 任意维向量点乘.
 *
 * \param dim   维数。
 * \param vec1	向量1。
 * \param vec2  向量2。
 * \return		点乘结果。
 */
float_def vec_dot(int dim, float_def* vec1, float_def* vec2);

/**
 * \brief 向量点乘.
 * 
 * \param vec1	向量1。
 * \param vec2  向量2。
 * \return		点乘结果。
 */
float_def vec3_dot(float_def vec1[3], float_def vec2[3])
{
	return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

/**
 * \brief		3维向量叉乘。
 *
 * \param vec1	3维向量。
 * \param vec2	3维向量。
 * \param vec3	3维向量。
 */
void vec3_cross(float_def vec1[3], float_def vec2[3], float_def vec3[3])
{
	vec3[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2];
	vec3[1] = -(vec1[0] * vec2[2] - vec2[0] * vec1[2]);
	vec3[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1];
	return;
}

/**
 * \brief		3维向量乘以一个数值。
 *
 * \param vec1
 * \param value
 * \param vec2
 */
void vec3_mult_value(float_def vec1[3], float_def value, float_def vec2[3])
{
	vec2[0] = vec1[0] * value;
	vec2[1] = vec1[1] * value;
	vec2[2] = vec1[2] * value;
	return;
}

/**
 * \brief 向量单位化。
 *
 * \param vec1	输入向量。
 * \param vec2	单位化后的输出向量。
 * \return 返回值0执行成功，返回值非0执行失败。
 * \retval 1 输入向量为0。
 */
int vec3_unit(float_def* vec1, float_def* vec2)
{
	float_def tmp = vec_norm(3, vec1);
	if (tmp < 1.0e-5)return 1;
	vec2[0] = vec1[0] / tmp;
	vec2[1] = vec1[1] / tmp;
	vec2[2] = vec1[2] / tmp;
	return 0;
}


/**
 * \brief		给一个矩阵分配内存，返回矩阵的二级指针。务必检查返回值，返回值为NULL则分配内存失败。
 *
 * \param row	矩阵的行数。
 * \param col	矩阵的列数。
 * \return		返回NULL，则内存分配失败。
 */
float_def** dmatrix(int row, int col)
{
	int i;
	float_def** m;
	m = (float_def**)malloc((size_t)(row) * sizeof(float_def*));
	if (!m)return NULL;
	m[0] = (float_def*)malloc((size_t)(row * col) * sizeof(float_def));
	if (!m[0])
	{
		free(m);
		return NULL;
	}
	for (i = 1; i < row; i++)m[i] = m[i - 1] + col;
	return m;
}


/**
 * \brief		释放float_def类型的矩阵。
 *
 * \param m		指向矩阵的指针。
 * \param row	矩阵的行数。
 * \param col	矩阵的列数。
 */
void free_dmatrix(float_def** m)
{
	free((FREE_ARG*)(m[0]));
	free((FREE_ARG*)(m));
	return;
}

/**
 * \brief		用于复制dmatrix函数创建的矩阵。，矩阵a的元素赋值给矩阵b。
 *
 * \param row	矩阵行数。
 * \param col	矩阵列数。
 * \param a		源矩阵。
 * \param b		被赋值的矩阵。
 * \attention	注意只能用于复制dmatrix函数创建的矩阵。
 */
void dmatrix_cpy(int m, int n, float_def** a, float_def** b)
{
	int row, col;
	for (row = 0; row < m; row++)
		for (col = 0; col < n; col++)
			b[row][col] = a[row][col];
	return;

}

/**
 * \brief		用于复制固定维数的数组型矩阵，二维数组指针需转化为一级指针，否则编译会报错。
 *
 * \param row	矩阵行数。
 * \param col	矩阵列数。
 * \param a		源矩阵。
 * \param b		被赋值的矩阵。
 * \attention	注意只能用于复制固定维数的数组型矩阵。
 */
void matrix_cpy(int m, int n, float_def* a, float_def* b)
{
	int row, col;
	for (row = 0; row < m; row++)
		for (col = 0; col < n; col++)
			b[row * n + col] = a[row * n + col];
	return;

}


/**
 * \brief		三阶数组矩阵的复制.
 * 
 * \param a		源矩阵。
 * \param b		被赋值的矩阵。
 * \attention   存储矩阵的内存块必须连续，适用于固定维数的矩阵。
 */
void matrix3_cpy(float_def a[][3], float_def b[][3])
{
	int row, col;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			b[row][col] = a[row][col];
		}
	}
	return;
}

/**
 * \brief		四阶数组型矩阵复制。
 * 
 * \param a		源矩阵。
 * \param b		被赋值的矩阵。
 * \attention   存储矩阵的内存块必须连续，适用于固定维数的矩阵或malloc分配内存的矩阵。
 */
void matrix4_cpy(float_def a[][4], float_def b[][4])
{
	int row,col;
	for (row = 0; row < 4; row++)
	{
		for (col = 0; col < 4; col++)
		{
			b[row][col] = a[row][col];
		}
	}
	return;
}


/**
 * \bief	3阶矩阵相加。
 * 
 * \param a	三阶矩阵。
 * \param b	三阶矩阵。
 * \param c	结果，三阶矩阵。
 */
void matrix3_add(float_def a[][3], float_def b[][3], float_def c[][3])
{
	int row, col;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			c[row][col] = a[row][col] + b[row][col];
		}
	}
	return;
}

/**
 * \brief		四阶矩阵加法。
 * 
 * \param a		矩阵a。
 * \param b		矩阵b。
 * \param c		矩阵c。
 */
void matrix4_add(float_def a[][4], float_def b[][4], float_def c[][4])
{
	int row, col;
	for (row = 0; row < 4; row++)
	{
		for (col = 0; col < 4; col++)
		{
			c[row][col] = a[row][col] + b[row][col];
		}
	}
	return;
}

/**
 * \brief		矩阵减法。
 * 
 * \param a		矩阵a。
 * \param b		矩阵b。
 * \param c		矩阵c。
 */
void matrix3_sub(float_def a[][3], float_def b[][3], float_def c[][3])
{
	int row, col;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			c[row][col] = a[row][col] - b[row][col];
		}
	}
	return;
}

/**
 * \brief		四阶矩阵减法。
 * 
 * \param a		矩阵a。
 * \param b		矩阵b。
 * \param c		矩阵c。
 */
void matrix4_sub(float_def a[][4], float_def b[][4], float_def c[][4])
{
	int row,col;
	for (row = 0; row < 4; row++)
	{
		for (col = 0; col < 4; col++)
		{
			c[row][col] = a[row][col] - b[row][col];
		}
	}
	return;
}

/**
 * \brief	3阶矩阵乘法。
 * 
 * \param a		矩阵a。
 * \param b		矩阵b。
 * \param c		矩阵c。
 */
void matrix3_mult(float_def a[][3], float_def b[][3], float_def c[][3])
{
	int row, col, k;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			c[row][col] = 0.0;
			for (k = 0; k < 3; k++)
			{
				c[row][col] = c[row][col] + a[row][k] * b[k][col];
			}
		}
	}
	return;
}

/**
 * \brief		4阶矩阵乘法.
 * 
 * \param a		矩阵a。
 * \param b		矩阵b。
 * \param c		矩阵c。
 */
void matrix4_mult(float_def a[][4], float_def b[][4], float_def c[][4])
{
	int row, col, k;
	for (row = 0; row < 4; row++)
	{
		for (col = 0; col < 4; col++)
		{
			c[row][col] = 0.0;
			for (k = 0; k < 4; k++)
			{
				c[row][col] = c[row][col] + a[row][k] * b[k][col];
			}
		}
	}
	return;
}

/**
 * \brief		齐次变换矩阵乘法（计算速度较快），该函数利用其特殊性，加快运算速度。
 * 
 * \param a		齐次变换矩阵a.
 * \param b		齐次变换矩阵b.
 * \param c		齐次变换矩阵c.
 */
void htm_matrix_mult(float_def a[][4], float_def b[][4], float_def c[][4])
{

	int row, col,k;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			c[row][col] = 0.0;
			for (k = 0; k < 3; k++)
			{
				c[row][col] = c[row][col] + a[row][k] * b[k][col];
			}
		}
	}

	for (row = 0; row < 3; row++)
	{
		c[row][3] = 0.0;
		for (col = 0; col < 3; col++)
			c[row][3] = c[row][3] + a[row][col] * b[col][3] ;
		c[row][3] = c[row][3] + a[row][3];
	}
	c[3][0] = 0.0;
	c[3][1] = 0.0;
	c[3][2] = 0.0;
	c[3][3] = 1.0;
	return;
}



/**
 * \brief		任意阶矩阵乘法，适用于dmatrix创建的矩阵乘法，不适用于固定维数的数组型矩阵乘法。
 *  
 * \param a
 * \param m
 * \param n
 * \param b
 * \param l
 * \param c
 */
void dmatrix_mult( float_def** a, int m, int n, const float_def** b, int l, float_def** c)
{
	int i, j, k;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < l; j++)
		{
			c[i][j] = 0.0;
			for (k = 0; k < n; k++)
			{
				c[i][j] = c[i][j] + a[i][k] * b[k][j];
			}
		}
	}
	return;
}

/**
 * \brief		任意阶矩阵乘法，适用于固定维数的数组型矩阵乘法。
 *
 * \param a		矩阵a。
 * \param m		矩阵a的行数。
 * \param n		矩阵a的列数。
 * \param b		矩阵b。
 * \param l		矩阵b的列数。
 * \param c		矩阵a*b的结果。
 */
void matrix_mult( float_def* a, int m, int n,const float_def* b, int l, float_def* c)
{
	int i, j, k;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < l; j++)
		{
			c[i * l + j] = 0.0;
			for (k = 0; k < n; k++)
			{
				c[i * l + j] = c[i * l + j] + a[i * n + k] * b[k * l + j];
			}
		}
	}
	return;
}


/**
 * \brief		3阶矩阵乘以一个数值。
 * 
 * \param a		矩阵a。
 * \param Value	数值。
 * \param c		矩阵c。
 */
void matrix3_mult_value(float_def a[][3], float_def Value, float_def c[][3])
{
	int row;
	int col;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			c[row][col] = a[row][col] * Value;
		}
	}
	return;
}

/**
 * \brief		4阶矩阵乘以一个数值。
 * 
 * \param a		矩阵a。
 * \param Value	数值。
 * \param c		矩阵c。
 */
void matrix4_mult_value(float_def a[][4], float_def Value, float_def c[][4])
{
	int row;
	int col;
	for (row = 0; row < 4; row++)
	{
		for (col = 0; col < 4; col++)
		{
			c[row][col] = a[row][col] * Value;
		}
	}
	return;
}


/**
 * \brief		3阶矩阵乘以一个向量。
 * 
 * \param R		3阶矩阵。
 * \param vec1	3维向量。
 * \param vec2	3维向量。
 */
void matrix3_mult_vec(float_def R[3][3], float_def vec1[3], float_def vec2[3])
{
	int row, col;
	for (row = 0; row < 3; row++)
	{
		vec2[row] = 0;
		for (col = 0; col < 3; col++)
		{
			vec2[row] = vec2[row] + R[row][col] * vec1[col];
		}
	}
	return;
}


/**
 * \brief		4阶矩阵乘以一个向量。
 *
 * \param R		4阶矩阵。
 * \param vec1	4维向量。
 * \param vec2	4维向量。
 */
void matrix4_mult_vec(float_def T[4][4], float_def vec1[4], float_def vec2[4])
{
	int row, col;
	for (row = 0; row < 4; row++)
	{
		vec2[row] = 0;
		for (col = 0; col < 4; col++)
		{
			vec2[row] = vec2[row] + T[row][col] * vec1[col];
		}
	}
	return;
}

/**
 * \brief		计算矩阵的转置，适用于固定维数的数组型矩阵（或内存连续的矩阵）。
 * 
 * \param a		矩阵a。
 * \param m		矩阵的行数。
 * \param n		矩阵的列数。
 * \param b		转置后的矩阵。
 */
void matrix_tranpose(float_def* a, int m, int n, float_def* b)
{
	int row, col;
	for (row = 0; row < m; row++)
	{
		for (col = 0; col < n; col++)
		{
			b[col * m + row] = a[row * n + col];
		}
	}
	return;
}


/**
 * \brief	计算两个向量的夹角，范围[0,pi]，单位弧度.
 * 
 * \param dim	向量的维度。
 * \param v1	向量1。
 * \param v2	向量2。
 * \return		两个向量的夹角，范围[0,pi],单位弧度
 */
float_def calc_vector_angle(int dim,float_def v1[3], float_def v2[3])
{
	float_def len1 = 0.0, len2 = 0.0, temp = 0.0;
	int i;
	for (i = 0; i < dim; i++)
	{
		len1 = len1 + v1[i]*v1[i];
		len2 = len2 + v2[i] * v2[i];
	}
	len1 = sqrt(len1);
	len2 = sqrt(len2);
	if (len1 < 1.0E-6 || len2 < 1.0E-6)
		return 0.0;
	else
	{
		temp = vec_dot(dim,v1, v2) / (len1 * len2);
		if (temp < -1.0)temp = -1.0;
		if (temp > 1.0)temp = 1.0;
		temp = acos(temp);
	}
	return temp;
}




/**
 * \brief	3阶矩阵求逆（仅适用于旋转矩阵求逆）.
 * 
 * \param R	旋转矩阵。
 * \param invR	旋转矩阵的逆。
 */
void rot_inv(float_def R[3][3], float_def invR[3][3])
{
	int row, col;
	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
		{
			invR[row][col] = R[col][row];
		}
	}
	return;
}


/**
 * \brief	旋转矩阵和位移构建齐次变换矩阵。
 * 
 * \param R	旋转矩阵。
 * \param p	位移。
 * \param T	齐次变换矩阵。
 */
void rp_trans(float_def R[3][3], float_def p[3], float_def T[4][4])
{
	int i;
	int j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			T[i][j] = R[i][j];
		}
	}
	T[0][3] = p[0];
	T[1][3] = p[1];
	T[2][3] = p[2];
	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
	return;
}

/**
 * \brief	从齐次变换矩阵中提取旋转矩阵和位移分量。
 * 
 * \param T	齐次变换矩阵。
 * \param R	旋转矩阵。
 * \param p	位移分量。
 */
void trans_rp(float_def T[4][4], float_def R[3][3], float_def p[3])
{
	int i;
	int j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			R[i][j] = T[i][j];
		}
	}
	p[0] = T[0][3];
	p[1] = T[1][3];
	p[2] = T[2][3];
	return;
}

/**
 * \brief	计算齐次变换矩阵的逆矩阵。
 * 
 * \param T		齐次变换矩阵。
 * \param InvT	齐次变换矩阵对应的逆矩阵。
 */
void trans_inv(float_def T[4][4], float_def InvT[4][4])
{
	float_def R[3][3];
	float_def invR[3][3];
	float_def p[3];
	float_def p2[3];
	trans_rp(T, R, p);
	rot_inv(R, invR);
	matrix3_mult_value(invR, -1.0, R);
	matrix3_mult_vec(R, p, p2);
	rp_trans(invR, p2, InvT);
	return;
}


/**
 * \brief		绕固定坐标系旋转rx,ry,rz，或绕当前动坐标系旋转rz,ry,rx。
 * 
 * \param rpy	绕固定坐标系的X,Y,Z轴旋转roll,pitch,yaw角度，或绕动坐标Z,Y,X,旋转yaw,pitch,roll,单位：弧度。
 * \param R		旋转矩阵。
 */
void rpy2r(float_def rpy[3], float_def R[3][3])
{
	float_def sr = sin(rpy[0]);
	float_def cr = cos(rpy[0]);
	float_def sp = sin(rpy[1]);
	float_def cp = cos(rpy[1]);
	float_def sy = sin(rpy[2]);
	float_def cy = cos(rpy[2]);
	R[0][0] = cy * cp;
	R[0][1] = cy * sp * sr - sy * cr;
	R[0][2] = cy * sp * cr + sy * sr;
	R[1][0] = sy * cp;
	R[1][1] = sy * sp * sr + cy * cr;
	R[1][2] = sy * sp * cr - cy * sr;
	R[2][0] = -sp;
	R[2][1] = cp * sr;
	R[2][2] = cp * cr;
	return;
}



/**
 * \brief		旋转矩阵计算绕固定坐标系的rpy角(X-Y-Z固定角roll,pitch,yaw),或者旋转顺序为绕当前动坐标系旋转XYZ序列（Z-Y-X欧拉角）。
 *
 * \param R		旋转矩阵。
 * \param rpy	rpy角,X-Y-Z固定角序列roll,pitch,yaw。或者Z-Y-X欧拉角序列roll,pitch,yaw，或单位：弧度。
 */
void r2rpy(float_def R[3][3], float_def rpy[3])
{
	float_def roll = 0.0;
	float_def pitch = 0.0;
	float_def yaw = 0.0;
	if (fabs(1.0 + R[2][0]) < ZERO_TOL)
	{
		yaw = 0;
		pitch = PM_PI / 2.0;
		roll = atan2(R[0][1], R[1][1]);

	}
	else if (fabs(1.0 - R[2][0]) < ZERO_TOL)
	{
		yaw = 0;
		pitch = -PM_PI / 2.0;
		roll = -atan2(R[0][1], R[1][1]);
	}
	else
	{
		pitch = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
		yaw = atan2(R[1][0]/cos(pitch), R[0][0]/cos(pitch));
		roll = atan2(R[2][1] / cos(pitch), R[2][2] / cos(pitch));
	}
	rpy[0] = roll;
	rpy[1] = pitch;
	rpy[2] = yaw;
	return;
}



/**
 * \brief 四元数转化为旋转矩阵。
 * 
 * \note 四元数分量顺序固定为 [qx, qy, qz, w]。
 * \note 输入 q 应为单位四元数；若不是单位四元数，请先归一化再调用。
 * \param q		四元数 [qx, qy, qz, w]。
 * \param R		旋转矩阵。
 */
void q2rot(float_def q[4],float_def R[3][3])
{
/*
	function R = q2rot(q)
	q=[q1,q2,q3,w]
		% Converting a quaternion q to
		% Rotation R from
		%
		%http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
		%
		%APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
		% Working Paper, Stanford AI Lab, 19791
		% by Eugene Salamin
		%
		%Mili Shah
		% Aug 25, 2011
*/
	R[0][0] = q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2];
	R[1][1] = q[3] * q[3] - q[0] * q[0] + q[1] * q[1] - q[2] * q[2];
	R[2][2] = q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2];

	R[0][1] = 2.0 * (-q[3] * q[2] + q[0] * q[1]);
	R[1][0] = 2.0 * (q[3] * q[2] + q[0] * q[1]);

	R[0][2] = 2.0 * (q[3] * q[1] + q[0] * q[2]);
	R[2][0] = 2.0 * (-q[3] * q[1] + q[0] * q[2]);

	R[1][2] = 2.0 * (-q[3] * q[0] + q[1] * q[2]);
	R[2][1] = 2.0 * (q[3] * q[0] + q[1] * q[2]);
	return;
}


/**
 * 
 * \brief 旋转矩阵转化为四元数。
 * 
 * \param R		旋转矩阵。
 * \param q		输出四元数 [qx, qy, qz, w]。
 * \note 输出四元数的分量顺序固定为 [qx, qy, qz, w]，并满足单位化要求。
 */
void rot2q(float_def R[3][3], float_def q[4])
{
	/*
	function q = rot2q(R)

		% Converting a rotation matrix R to
		% quaternion q from
		%
		%http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
	%
		%APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
		% Working Paper, Stanford AI Lab, 19791
		% by Eugene Salamin
		%
		%Mili Shah
		% Aug 25, 2011
	*/

	int i, in;
	q[3] = 0.5 * sqrt(1 + R[0][0] + R[1][1] + R[2][2]);
	q[0] = 0.5 * sqrt(1 + R[0][0] - R[1][1] - R[2][2]);
	q[1] = 0.5 * sqrt(1 - R[0][0] + R[1][1] - R[2][2]);
	q[2] = 0.5 * sqrt(1 - R[0][0] - R[1][1] + R[2][2]);

	for (i = 0; i < 3; i++)
	{
		if (q[i + 1] < q[i])
		{			
			in = i + 1;
		}
		else
		{		
			in = i;
		}
	}
	if (in == 0)
	{
		q[1] = 0.25 / q[0] * (R[0][1] + R[1][0]);
		q[2] = 0.25 / q[0] * (R[0][2] + R[2][0]);
		q[3] = 0.25 / q[0] * (R[2][1] - R[1][2]);
	}
	else if (in == 1)
	{
		q[0] = 0.25 / q[1] * (R[0][1] + R[1][0]);
		q[2] = 0.25 / q[1] * (R[1][2] + R[2][1]);
		q[3] = 0.25 / q[1] * (R[0][2] - R[2][0]);
	}
	else if (in == 2)
	{
		q[0] = 0.25 / q[2] * (R[0][2] + R[2][0]);
		q[1] = 0.25 / q[2] * (R[1][2] + R[2][1]);
		q[3] = 0.25 / q[2] * (R[1][0] - R[0][1]);
	}
	else if (in == 3)
	{
		q[0] = 0.25 / q[3] * (R[2][1] - R[1][2]);
		q[1] = 0.25 / q[3] * (R[0][2] - R[2][0]);
		q[2] = 0.25 / q[3] * (R[1][0] - R[0][1]);
	}
	return;
}

void quat2rot(float_def quat[4], float_def R[3][3])
{
	float_def qn[4];
	float_def qxyzw[4];
	float_def norm = quat_norm(quat);

	if (norm < ZERO_TOL)
	{
		R[0][0] = 1.0; R[0][1] = 0.0; R[0][2] = 0.0;
		R[1][0] = 0.0; R[1][1] = 1.0; R[1][2] = 0.0;
		R[2][0] = 0.0; R[2][1] = 0.0; R[2][2] = 1.0;
		return;
	}

	qn[0] = quat[0] / norm;
	qn[1] = quat[1] / norm;
	qn[2] = quat[2] / norm;
	qn[3] = quat[3] / norm;

	qxyzw[0] = qn[1];
	qxyzw[1] = qn[2];
	qxyzw[2] = qn[3];
	qxyzw[3] = qn[0];

	q2rot(qxyzw, R);
}

void rot2quat(float_def R[3][3], float_def quat[4])
{
	float_def qxyzw[4];

	rot2q(R, qxyzw);
	quat[0] = qxyzw[3];
	quat[1] = qxyzw[0];
	quat[2] = qxyzw[1];
	quat[3] = qxyzw[2];
}

void rpy2quat(float_def rpy[3], float_def quat[4])
{
	float_def halfRoll = 0.5 * rpy[0];
	float_def halfPitch = 0.5 * rpy[1];
	float_def halfYaw = 0.5 * rpy[2];
	float_def sr = sin(halfRoll);
	float_def cr = cos(halfRoll);
	float_def sp = sin(halfPitch);
	float_def cp = cos(halfPitch);
	float_def sy = sin(halfYaw);
	float_def cy = cos(halfYaw);
	float_def qtmp[4];

	/*
	 * 固定坐标系 XYZ(RPY) 与动坐标系 ZYX 等价：
	 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
	 * 单位四元数 [w, x, y, z] 闭式表达：
	 * w = cr*cp*cy + sr*sp*sy
	 * x = sr*cp*cy - cr*sp*sy
	 * y = cr*sp*cy + sr*cp*sy
	 * z = cr*cp*sy - sr*sp*cy
	 */
	qtmp[0] = cr * cp * cy + sr * sp * sy;
	qtmp[1] = sr * cp * cy - cr * sp * sy;
	qtmp[2] = cr * sp * cy + sr * cp * sy;
	qtmp[3] = cr * cp * sy - sr * sp * cy;

	if (quat_normalize(qtmp, quat) != 0)
	{
		quat[0] = 1.0;
		quat[1] = 0.0;
		quat[2] = 0.0;
		quat[3] = 0.0;
	}
}

void quat2rpy(float_def quat[4], float_def rpy[3])
{
	float_def qn[4];
	float_def sinp;

	if (quat_normalize(quat, qn) != 0)
	{
		rpy[0] = 0.0;
		rpy[1] = 0.0;
		rpy[2] = 0.0;
		return;
	}

	/*
	 * 单位四元数 [w, x, y, z] 直接求固定坐标系 XYZ(RPY) 角，
	 * 等价于动坐标系 ZYX(yaw,pitch,roll) 角：
	 * roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
	 * pitch = asin (2*(w*y - z*x))
	 * yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
	 */
	rpy[0] = atan2(2.0 * (qn[0] * qn[1] + qn[2] * qn[3]),
					 1.0 - 2.0 * (qn[1] * qn[1] + qn[2] * qn[2]));

	sinp = 2.0 * (qn[0] * qn[2] - qn[3] * qn[1]);
	if (sinp > 1.0)
	{
		sinp = 1.0;
	}
	else if (sinp < -1.0)
	{
		sinp = -1.0;
	}
	rpy[1] = asin(sinp);

	rpy[2] = atan2(2.0 * (qn[0] * qn[3] + qn[1] * qn[2]),
					 1.0 - 2.0 * (qn[2] * qn[2] + qn[3] * qn[3]));
}

void quat_copy(const float_def src[4], float_def dst[4])
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

float_def quat_norm(const float_def q[4])
{
	return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

int quat_normalize(const float_def q[4], float_def out[4])
{
	float_def norm = quat_norm(q);
	if (norm < ZERO_TOL)
	{
		return 1;
	}
	out[0] = q[0] / norm;
	out[1] = q[1] / norm;
	out[2] = q[2] / norm;
	out[3] = q[3] / norm;
	return 0;
}

float_def quat_dot(const float_def q0[4], const float_def q1[4])
{
	return q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
}

void quat_scale(const float_def q[4], float_def scale, float_def out[4])
{
	out[0] = q[0] * scale;
	out[1] = q[1] * scale;
	out[2] = q[2] * scale;
	out[3] = q[3] * scale;
}

void quat_conjugate(const float_def q[4], float_def out[4])
{
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

void quat_multiply(const float_def q0[4], const float_def q1[4], float_def out[4])
{
	out[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3];
	out[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2];
	out[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0] + q0[3] * q1[1];
	out[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1] + q0[3] * q1[0];
}

int quat_slerp(const float_def q0[4], const float_def q1[4], float_def s, float_def out[4])
{
	float_def qa[4];
	float_def qb[4];
	float_def dot;
	float_def theta0;
	float_def sinTheta0;
	float_def theta;
	float_def w0;
	float_def w1;
	float_def qTmp[4];

	if (quat_normalize(q0, qa) != 0 || quat_normalize(q1, qb) != 0)
	{
		return 1;
	}

	dot = quat_dot(qa, qb);
	if (dot < -1.0)
	{
		dot = -1.0;
	}
	else if (dot > 1.0)
	{
		dot = 1.0;
	}

	if (dot < 0.0)
	{
		quat_scale(qb, -1.0, qb);
		dot = -dot;
	}

	if (s < 0.0)
	{
		s = 0.0;
	}
	else if (s > 1.0)
	{
		s = 1.0;
	}

	if (dot > 0.9995)
	{
		qTmp[0] = qa[0] + s * (qb[0] - qa[0]);
		qTmp[1] = qa[1] + s * (qb[1] - qa[1]);
		qTmp[2] = qa[2] + s * (qb[2] - qa[2]);
		qTmp[3] = qa[3] + s * (qb[3] - qa[3]);
		return quat_normalize(qTmp, out);
	}

	theta0 = acos(dot);
	sinTheta0 = sin(theta0);
	if (fabs(sinTheta0) < ZERO_TOL)
	{
		quat_copy(qa, out);
		return 0;
	}

	theta = theta0 * s;
	w0 = sin(theta0 - theta) / sinTheta0;
	w1 = sin(theta) / sinTheta0;

	out[0] = w0 * qa[0] + w1 * qb[0];
	out[1] = w0 * qa[1] + w1 * qb[1];
	out[2] = w0 * qa[2] + w1 * qb[2];
	out[3] = w0 * qa[3] + w1 * qb[3];

	return quat_normalize(out, out);
}

float_def clamp_value(float_def value, float_def low, float_def high)
{
	if (value < low)
	{
		return low;
	}
	if (value > high)
	{
		return high;
	}
	return value;
}

float_def wrap_to_pi(float_def angle)
{
	while (angle > PM_PI)
	{
		angle -= 2.0 * PM_PI;
	}
	while (angle < -PM_PI)
	{
		angle += 2.0 * PM_PI;
	}
	return angle;
}


/**
 * \brief		rpy角转化为齐次变换矩阵。
 * 
 * \param rpy	rpy角,X-Y-Z固定角序列roll,pitch,yaw。或者Z-Y-X欧拉角序列roll,pitch,yaw，，单位：弧度。
 * \param T		齐次变换矩阵。
 */
void rpy2tr(float_def rpy[3], float_def T[4][4])
{
	int i, j;
	float_def R[3][3] = { {0} };
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			T[i][j] = 0.0;
	rpy2r(rpy, R);
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			T[i][j] = R[i][j];
	T[3][3] = 1.0;
	return;
}

/**
 * \brief		rpy位姿描述角转化为齐次变换矩阵。
 *
 * \param xyzrpy	坐标x,y,z和绕固定坐标系的roll,pitch,yaw角度，单位：弧度。
 * \param T		齐次变换矩阵。
 */
void xyzrpy2tr(float_def xyzrpy[6], float_def T[4][4])
{
	int i, j;
	float_def R[3][3] = { {0} };
	float_def rpy[3] = { xyzrpy[3],xyzrpy[4],xyzrpy[5] };
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			T[i][j] = 0.0;
	rpy2r(rpy, R);
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			T[i][j] = R[i][j];
	T[0][3] = xyzrpy[0]; //* R[0][0] + xyzrpy[1] * R[0][1] + xyzrpy[2] * R[0][2];//xyzrpy[2] * (sin(rpy[0]) * sin(rpy[2]) + cos(rpy[0]) * cos(rpy[2]) * sin(rpy[1])) - xyzrpy[1] * (cos(rpy[0]) * sin(rpy[2]) - cos(rpy[2]) * sin(rpy[0]) * sin(rpy[1])) + xyzrpy[0] * cos(rpy[1]) * cos(rpy[2]);
	T[1][3] = xyzrpy[1]; //* R[1][0] + xyzrpy[1] * R[1][1] + xyzrpy[2] * R[1][2];//xyzrpy[0] * (cos(rpy[0]) * cos(rpy[2]) + sin(rpy[0]) * sin(rpy[1]) * sin(rpy[2])) - xyzrpy[2] * (cos(rpy[2]) * sin(rpy[0]) - cos(rpy[0]) * sin(rpy[1]) * sin(rpy[2])) + xyzrpy[0] * cos(rpy[1]) * sin(rpy[2]);
	T[2][3] = xyzrpy[2]; //* R[2][0] + xyzrpy[1] * R[2][1] + xyzrpy[2] * R[2][2];// xyzrpy[2] * cos(rpy[0]) * cos(rpy[1]) - xyzrpy[0] * sin(rpy[1]) + xyzrpy[1] * cos(rpy[1]) * sin(rpy[0]);
	T[3][3] = 1.0;
	return;
}


/**
 * \brief		齐次矩阵计算绕固定坐标系的位姿和rpy角。
 *
 * \param T		齐次矩阵。
 * \param rpy	rpy角,单位：弧度。
 */
void tr2xyzrpy(float_def T[4][4], float_def xyzrpy[6])
{
	float_def R[3][3] =
        {
	{1,0,0},
	{0,1,0},
	{0,0,1} 
        };
        
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			R[i][j] = T[i][j];
	r2rpy(R, &xyzrpy[3]);
	xyzrpy[0] = T[0][3];
	xyzrpy[1] = T[1][3];
	xyzrpy[2] = T[2][3];
	return;
}



/**
 * \brief 绕x轴旋转的矩阵.
 * \param rx	绕x轴旋转的角度，单位deg.
 * \param R		绕x轴旋转的旋转矩阵.
 */
void rotx(float_def rx, float_def R[3][3])
{
	R[0][0] = 1.0;
	R[1][0] = 0.0;
	R[2][0] = 0.0;
	R[0][1] = 0.0;
	R[1][1] = cos(rx * PM_PI / 180.0);
	R[2][1] = sin(rx * PM_PI / 180.0);
	R[0][2] = 0.0;
	R[1][2] = -sin(rx * PM_PI / 180.0);
	R[2][2] = cos(rx * PM_PI / 180.0);
	return;
}

/**
 * \brief 绕y轴旋转的矩阵.
 * \param ry	绕y轴旋转的角度，单位deg.
 * \param R		绕y轴旋转的旋转矩阵.
 */
void roty(float_def ry, float_def R[3][3])
{
	R[0][0] = cos(ry * PM_PI / 180.0);
	R[1][0] = 0.0;
	R[2][0] = -sin(ry * PM_PI / 180.0);
	R[0][1] = 0.0;
	R[1][1] = 1.0;
	R[2][1] = 0.0;
	R[0][2] = sin(ry * PM_PI / 180.0);
	R[1][2] = 0.0;
	R[2][2] = cos(ry * PM_PI / 180.0);
	return;
}


/**
 * \brief 绕z轴旋转的矩阵.
 * \param rz	绕z轴旋转的角度，单位deg.
 * \param R		绕z轴旋转的旋转矩阵.
 */
void rotz(float_def rz, float_def R[3][3])
{
	R[0][0] = cos(rz * PM_PI / 180.0);
	R[1][0] = sin(rz * PM_PI / 180.0);
	R[2][0] = 0.0;
	R[0][1] = -sin(rz * PM_PI / 180.0);
	R[1][1] = cos(rz * PM_PI / 180.0);
	R[2][1] = 0.0;
	R[0][2] = 0.0;
	R[1][2] = 0.0;
	R[2][2] = 1.0;
	return;
}

/**
 * \brief 把XYZ欧拉角转换为旋转矩阵，XYZ欧拉角定义为：分别绕动坐标系的X,Y,Z轴旋转角度rx,ry,rz,对应的旋转矩阵为.
 * Rxyz =Rotx*Roty*Rotz=
 * [                           cos(ry)*cos(rz),                          -cos(ry)*sin(rz),          sin(ry)]
 * [ cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry), cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz), -cos(ry)*sin(rx)]
 * [ sin(rx)*sin(rz) - cos(rx)*cos(rz)*sin(ry), cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz),  cos(rx)*cos(ry)]
 * \param Rx	绕动坐标系X轴旋转的角度，单位度。
 * \param Ry	绕动坐标系Y轴旋转的角度，单位度。
 * \param Rz	绕动坐标系Z轴旋转的角度，单位度。
 * \param R		输出的旋转矩阵。
 * \return		返回0执行成功，非零执行失败。
 */
int xyzEulerAngle2Rotation(float_def Rx, float_def Ry, float_def Rz, float_def R[3][3])
{
	float_def rx = Rx * PM_PI / 180.0;
	float_def ry = Ry * PM_PI / 180.0;
	float_def rz = Rz * PM_PI / 180.0;
	float_def crx = cos(rx);
	float_def cry = cos(ry);
	float_def srx = sin(rx);
	float_def sry = sin(ry);
	float_def crz = cos(rz);
	float_def srz = sin(rz);
	R[0][0] = cry * crz;
	R[0][1] = -cry * srz;
	R[0][2] = sry;
	R[1][0] = crx * srz + crz * srx * sry;
	R[1][1] = crx * crz - srx * sry * srz;
	R[1][2] = -cry * srx;
	R[2][0] = srx * srz - crx * crz * sry;
	R[2][1] = crz * srx + crx * sry * srz;
	R[2][2] = crx * cry;
	return 0;
}

/**
 * \brief 给定旋转矩阵，求解XYZ欧拉角，XYZ欧拉角定义为：分别绕动坐标系的X,Y,Z轴旋转角度rx,ry,rz.
 * 
 * \param R	旋转矩阵。
 * \param Rx	绕动坐标系X轴旋转的角度，单位度。求解范围Rx,Rz（-pi,pi],Ry[-pi/2,pi/2]
 * \param Ry	绕动坐标系Y轴旋转的角度，单位度。
 * \param Rz	绕动坐标系Z轴旋转的角度，单位度。
 * \return		返回0执行成功，非零执行失败。
 */
int rotation2xyzEulerAngle(float_def R[3][3], float_def* Rx, float_def* Ry, float_def* Rz)
{
	if (fabs(R[0][2] - 1.0) < ZERO_TOL)
	{
		*Rx = 0.0;
		*Ry = 90.0;
		*Rz = atan2(R[1][0], R[1][1]) * 180.0 / PM_PI;
	}
	else if (fabs(R[0][2] + 1.0) < ZERO_TOL)
	{
		*Rx = 0.0;
		*Ry = -90.0;
		*Rz = atan2(R[1][0], R[1][1]) * 180.0 / PM_PI;
	}
	else
	{
		*Rx = -atan2(R[1][2], R[2][2]) * 180.0 / PM_PI;	
		*Ry = asin(R[0][2]) * 180.0 / PM_PI;// atan2(R[0][2], sqrt(R[0][0] * R[0][0] + R[0][1] * R[0][1])) * 180.0 / PM_PI;
		*Rz = -atan2(R[0][1], R[0][0]) * 180.0 / PM_PI;
	}
	return 0;
}

int xyzEulerAngle2quat(float_def Rx, float_def Ry, float_def Rz, float_def quat[4])
{
	float_def R[3][3];
	xyzEulerAngle2Rotation(Rx, Ry, Rz, R);
	rot2quat(R, quat);
	return 0;
}

int quat2xyzEulerAngle(float_def quat[4], float_def* Rx, float_def* Ry, float_def* Rz)
{
	float_def R[3][3];
	quat2rot(quat, R);
	return rotation2xyzEulerAngle(R, Rx, Ry, Rz);
}

/**
 * \brief 把刚体运动x,y,z,rx,ry,rz笛卡尔坐标描述转换为旋转矩阵描述，刚体运动为transl(x)*transl(y)*transl(z)*rotx(rx)*roty(ry)*rotz(rz).
transl(x)*transl(y)*transl(z)*rotx(rx)*roty(ry)*rotz(rz)=
[                           cos(RY)*cos(RZ),                          -cos(RY)*sin(RZ),          sin(RY), Xt]
[ cos(RX)*sin(RZ) + cos(RZ)*sin(RX)*sin(RY), cos(RX)*cos(RZ) - sin(RX)*sin(RY)*sin(RZ), -cos(RY)*sin(RX), Yt]
[ sin(RX)*sin(RZ) - cos(RX)*cos(RZ)*sin(RY), cos(RZ)*sin(RX) + cos(RX)*sin(RY)*sin(RZ),  cos(RX)*cos(RY), Zt]
[                                         0,                                         0,                0,  1]
 * \param xe	迪卡尔坐标，单位度。
 * \param T		齐次矩阵。
 * \return		返回0执行成功，非零执行失败。
 */
int xyzEulerAngle2tr(float_def xe[6],float_def T[4][4])
{
	float_def R[3][3] = 
        {
            {1,0,0},
            {0,1,0},
            {0,0,1} 
        };
	int i,j;
	xyzEulerAngle2Rotation(xe[3], xe[4], xe[5], R);
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			T[i][j] = R[i][j];
	T[0][3] = xe[0];
	T[1][3] = xe[1];
	T[2][3] = xe[2];
	T[3][3] = 1.0;
	return 0;
}

/**
 * \brief 把刚体运动旋转矩阵描述转换为x,y,z,rx,ry,rz笛卡尔坐标描述，刚体运动为transl(x)*transl(y)*transl(z)*rotx(rx)*roty(ry)*rotz(rz).
 * \param T		齐次矩阵。
 * \param xe	迪卡尔坐标，单位度。
 * \return		返回0执行成功，非零执行失败。
 */
int tr2xyzEulerAngle(float_def T[4][4], float_def xe[6])
{
	float_def R[3][3] = { {0.0 }};
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			R[i][j] = T[i][j];
	rotation2xyzEulerAngle(R, &xe[3], &xe[4], &xe[5]);
	xe[0] = T[0][3];
	xe[1] = T[1][3];
	xe[2] = T[2][3];
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//旋量理论相关函数实现，参考《Modern Robotics Mechanics, Planning, and Control》


/**
 * \brief 由轴方向和轴上一点计算单位运动旋量S=[w w x q]^T（即单位螺旋轴skew），
 *  
 * \param type	轴的类型：0--旋转轴，1--移动轴。
 * \param vec	轴的方向矢量
 * \param q
 * \param twist
 * \return		返回值0执行成功，返回值非0执行失败。
 * \retval 1	输入向量为0。
 */
void vec3_twist(int type, float_def vec[3], float_def q[3], float_def twist[6])
{
	int i;
	float_def v[3] = { 0 };
	
	if (type == 0)
	{
		vec3_cross(q, vec,v);
		for (i = 0; i < 3; i++)
		{
			twist[i] = vec[i];
			twist[i + 3] = v[i];
			
		}
	}
	else
	{
		for (i = 0; i < 3; i++)
		{			
			twist[i] = 0.0;
			twist[i + 3] = vec[i];
		}
	}
	return ;
}

/**
 * \brief 把一个3维向量转化为3x3反对称矩阵so3.
 * 
 * \param vec	3维向量
 * \param hatw	反对称矩阵。
 */
void vec_so3(float_def vec[3], float_def hatw[3][3])
{
	hatw[0][0] = 0.0;
	hatw[0][1] = -vec[2];
	hatw[0][2] = vec[1];

	hatw[1][0] = vec[2];
	hatw[1][1] = 0.0;
	hatw[1][2] = -vec[0];

	hatw[2][0] = -vec[1];
	hatw[2][1] = vec[0];
	hatw[2][2] = 0.0;
	return;
}

/**
*@brief			3x3反对称矩阵对应的三维向量.
*@param[in]		so3Mat		反对称矩阵.
*@param[out]	omg			3维向量.
*@return		无返回值.
*@note:
*@warning:
*/
void so3_vec(float_def so3Mat[3][3], float_def omg[3])
{
	omg[0] = so3Mat[2][1]; omg[1] = so3Mat[0][2]; omg[2] = so3Mat[1][0];
	return;
}

/**
*@brief 计算6维运动旋量对应的se(3)变换矩阵htwist。
*@param[in]		V		一个6维运动旋量.
*@param[out]	se3Mat	对应的 se(3) 矩阵.
*@return		无返回值.
*@note:
*@warning:
*/
void twist_se3(float_def V[6], float_def se3Mat[4][4])
{
	float_def so3Mat[3][3];
	float_def omg[3];
	int i;
	int j;
	omg[0] = V[0];
	omg[1] = V[1];
	omg[2] = V[2];
	vec_so3(omg, so3Mat);
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			se3Mat[i][j] = so3Mat[i][j];
		}

	}
	se3Mat[0][3] = V[3];
	se3Mat[1][3] = V[4];
	se3Mat[2][3] = V[5];
	se3Mat[3][0] = 0.0;
	se3Mat[3][1] = 0.0;
	se3Mat[3][2] = 0.0;
	se3Mat[3][3] = 0.0;
	return;
}

/**
*@brief Description:Computes the 6-vector twist corresponding to an se(3) matrix se3mat [V].
*@param[in]		se3Mat			an se(3) matrix.
*@param[out]	V				he 6-vector twist.
*@return		No return value.
*@note:
*@warning:
*/
void se3_twist(float_def se3Mat[4][4], float_def V[6])
{
	V[0] = se3Mat[2][1];
	V[1] = se3Mat[0][2];
	V[2] = se3Mat[1][0];
	V[3] = se3Mat[0][3];
	V[4] = se3Mat[1][3];
	V[5] = se3Mat[2][3];
	return;
}


/**
 * \brief  由给定ω,θ生成为反对称矩阵指数 exp(ωθ),即旋转矩阵
 * 
 * \param vec	3维向量
 * \param theta	关节量。
 * \param R		姿态矩阵。
 */
void vec_exp3(float_def vec[3], float_def theta, float_def exp3[3][3])
{
	float_def tmp1[3][3];
	float_def tmp2[3][3];
	float_def tmp3[3][3];
	float_def I33[3][3] = {
	{1,0,0},
	{0,1,0},
	{0,0,1} };
	float_def s = sin(theta);
	float_def c = 1.0-cos(theta);
	float_def skew[3][3];

	vec_so3(vec, skew);
	matrix3_mult_value(skew, s, tmp1);
	matrix3_add(I33, tmp1, tmp2);

	matrix3_mult(skew,skew,tmp1);
	matrix3_mult_value(tmp1, c, tmp3);
	matrix3_add(tmp2, tmp3, exp3);
	return ;
}



/**
*@brief			提取指数坐标hatw*theta对应的轴线方向向量和角度。
*				
*@param[in]		expc3       旋转的指数坐标。
*@param[out]	omghat		旋转轴单位向量 .
*@param[out]	theta		旋转角度.
*@return		No return value.
*@note:
*@warning:
*/
void expc3_ang3(float_def expc3[3], float_def omg[3], float_def* theta)
{
	int i;

	*theta = sqrt(expc3[0] * expc3[0] + expc3[1] * expc3[1] + expc3[2] * expc3[2]);
	if (*theta < 1.0e-6)
	{
		omg[0] = 0.0;
		omg[1] = 0.0;
		omg[2] = 0.0;
		*theta = 0.0;
		return;
	}
	for (i = 0; i < 3; i++)
	{
		omg[i] = expc3[i] / (*theta);
	}
	return;
}

/**
*@brief			计算反堆成矩阵的矩阵指数.
* 
*@param[in]		so3Mat		[omghat]*theta,matrix exponential of so3mat in so(3).
*@param[out]	R			旋转矩阵 R in SO(3).
*@return		No return value.
*@note:
*@warning:
*/
void matrix_exp3(float_def so3Mat[3][3], float_def R[3][3])
{
	float_def omgtheta[3];
	float_def omghat[3] = { 0 };
	float_def theta = 0;
	int i;
	int j;
	float_def MatI3[3][3] =
	{
		{1,0,0},
		{0,1,0},
		{0,0,1}
	};
	so3_vec(so3Mat, omgtheta);
	expc3_ang3(omgtheta, omghat, &theta);
	if (theta < 1.0e-6)
	{
		matrix3_cpy(MatI3, R);
		return;
	}
	else
	{

		//calculate formula(3.51) in book [modern robotics : mechanics,planning,and control]
		float_def omgmat[3][3];
		float_def temp[3][3];
		matrix3_mult_value(so3Mat, 1.0 / theta, omgmat);
		matrix3_mult(omgmat, omgmat, temp);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				R[i][j] = MatI3[i][j] + sin(theta) * so3Mat[i][j] / theta + (1 - cos(theta)) * temp[i][j];
			}
		}
	}
	return;
}

/**
*@brief			计算旋转矩阵的对数，即由旋转矩阵求对应的反对称矩阵.
*@param[in]		R		the rotation matrix.
*@param[out]	so3Mat	matrix logarithm.
*@return		No return value.
*@note:
*@warning:
*/
void matrix_log3(float_def R[3][3], float_def so3Mat[3][3])
{
	float_def omg[3] = { 0 };
	float_def acosinput = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0;
	if (fabs(acosinput - 1.0) < 1.0e-6)
	{
		memset(so3Mat, 0, 9 * sizeof(float_def));
	}
	else if (acosinput <= -1.0)
	{
		if ((1.0 + R[2][2]) >= 1.0e-6)
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[0][2];
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[1][2];
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * (1.0 + R[2][2]);
		}
		else if ((1.0 + R[1][1] >= 1.0e-6))
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[0][1];
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * (1.0 + R[1][1]);
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[2][1];
		}
		else
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * (1.0 + R[0][0]);
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[1][0];
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[2][0];
		}
		omg[0] = PM_PI * omg[0];
		omg[1] = PM_PI * omg[1];
		omg[2] = PM_PI * omg[2];
		vec_so3(omg, so3Mat);
	}
	else
	{
		int i;
		int j;
		float_def theta = acos(acosinput);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				so3Mat[i][j] = 1.0 / (2.0 * sin(theta)) * (R[i][j] - R[j][i]) * theta;
			}
		}
	}

	return;
}

/**
*@brief			Description: Computes the homogeneous transformation matrix T in SE(3) corresponding to
*				the matrix exponential of se3mat in se(3).
*@param[in]		se3mat			the matrix exponential of se3mat in se(3).
*@param[out]	T				the homogeneous transformation matrix T in SE(3).
*@return		No return value.
*@note:
*@warning:
*/
void matrix_exp6(float_def se3Mat[4][4], float_def T[4][4])
{
	int i;
	int j;
	float_def so3mat[3][3];
	float_def omgmat[3][3];
	float_def temp[3][3];
	float_def Gtheta[3][3];
	float_def p[3];
	float_def v[3];
	float_def omgtheta[3];
	float_def omghat[3];
	float_def theta;
	float_def MatI3[3][3] =
	{
		{1.0,0.0,0.0},
		{0.0,1.0,0.0},
		{0.0,0.0,1.0}
	};
	trans_rp(se3Mat, so3mat, p);//extracts so3mat from se3mat
	so3_vec(so3mat, omgtheta);
	if (vec_norm(3,omgtheta) < 1.0e-6)
	{
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				T[i][j] = MatI3[i][j];
			}
		}
		T[0][3] = se3Mat[0][3];
		T[1][3] = se3Mat[1][3];
		T[2][3] = se3Mat[2][3];
		T[3][0] = 0.0;
		T[3][1] = 0.0;
		T[3][2] = 0.0;
		T[3][3] = 1.0;
	}
	else
	{
		expc3_ang3(omgtheta, omghat, &theta);
		matrix_exp3(so3mat, temp);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				T[i][j] = temp[i][j];
			}
		}
		matrix3_mult_value(so3mat, 1.0 / theta, omgmat);
		matrix3_mult(omgmat, omgmat, temp);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				Gtheta[i][j] = MatI3[i][j] * theta + (1.0 - cos(theta)) * omgmat[i][j] + (theta - sin(theta)) * temp[i][j];
			}
		}
		v[0] = p[0] / theta;
		v[1] = p[1] / theta;
		v[2] = p[2] / theta;
		matrix3_mult_vec(Gtheta, v, p);
		T[0][3] = p[0]; T[1][3] = p[1]; T[2][3] = p[2];
		T[3][0] = 0.0;
		T[3][1] = 0.0;
		T[3][2] = 0.0;
		T[3][3] = 1.0;
	}
	return;
}


/**
*@brief Description: Computes the matrix logarithm se3mat in se(3) of
the homogeneous transformation matrix T in SE(3)
*@param[in]		T			the homogeneous transformation matrix.
*@param[out]	se3Mat		the matrix logarithm of T.
*@return		No return value.
*@note:
*@warning:
*/
void matrix_log6(float_def T[4][4], float_def se3Mat[4][4])
{
	int i;
	int j;
	float_def R[3][3];
	float_def so3mat[3][3];
	float_def p[3];
	trans_rp(T, R, p);
	matrix_log3(R, so3mat);
	int flag = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if (fabs(so3mat[i][j]) < 1.0e-6)
			{
				continue;
			}
			else
			{
				flag = 1;
				break;
			}
		}
	}
	if (!flag)
	{
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				se3Mat[i][j] = 0.0;
			}

		}
		se3Mat[0][3] = T[0][3];
		se3Mat[1][3] = T[1][3];
		se3Mat[2][3] = T[2][3];
		se3Mat[3][0] = 0.0;
		se3Mat[3][1] = 0.0;
		se3Mat[3][2] = 0.0;
		se3Mat[3][3] = 0.0;
	}
	else
	{
		float_def MatI3[3][3] =
		{
          	{1,0,0},
          	{0,1,0},
          	{0,0,1} 
		};
		float_def v[3];
		float_def temp[3][3];
		float_def InvGtheta[3][3];
		float_def theta = acos((R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				se3Mat[i][j] = so3mat[i][j];
			}
		}

		matrix3_mult(so3mat, so3mat, temp);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				InvGtheta[i][j] = MatI3[i][j] - 0.5 * so3mat[i][j] + (1.0 / theta - 0.5 * 1.0 / tan(0.5 * theta)) * temp[i][j] / theta;
			}
		}
		matrix3_mult_vec(InvGtheta, p, v);
		se3Mat[0][3] = v[0];
		se3Mat[1][3] = v[1];
		se3Mat[2][3] = v[2];
		se3Mat[3][0] = 0.0;
		se3Mat[3][1] = 0.0;
		se3Mat[3][2] = 0.0;
		se3Mat[3][3] = 0.0;
	}
	return;
}


/**
 * \brief	由向量和位移计算刚体变换矩阵。
 * 
 * \param type	类型：0--旋转轴，1--平移轴。
 * \param vec	轴的方向矢量。
 * \param q		轴上一点的坐标。
 * \param theta	关节量。
 * \param exp6	刚体的齐次变换矩阵。
 */
void vec_exp6(int type,float_def vec[3], float_def q[3],float_def theta, float_def exp6[4][4])
{
	int i,j;
	float_def twist[6];
	float_def v[3];
	float_def exp3[3][3];
	float_def I33[3][3] = {
	{1,0,0},
	{0,1,0},
	{0,0,1} };
	float_def tmp1[3][3] = { {0} };

	float_def tmpvec1[3] = { 0 };
	float_def tmpvec2[3] = { 0 };

	vec3_twist(type, vec, q, twist);
	v[0] = twist[3];
	v[1] = twist[4];
	v[2] = twist[5];
	memset(exp6, 0, 4 * 4 * sizeof(float_def));
	if (type == 0)
	{
		vec_exp3(vec, theta,exp3);	
		matrix3_sub(I33, exp3, tmp1);
		vec3_cross(vec, v, tmpvec1);
		matrix3_mult_vec(tmp1, tmpvec1, tmpvec2);

		matrix_mult(vec, 3, 1, vec, 3, (float_def *)tmp1);
		matrix3_mult_vec(tmp1, v, tmpvec1);
		vec3_mult_value(tmpvec1, theta, tmpvec1);

		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				exp6[i][j] = exp3[i][j];
		exp6[0][3] = tmpvec2[0] + tmpvec1[0];
		exp6[1][3] = tmpvec2[1] + tmpvec1[1];
		exp6[2][3] = tmpvec2[2] + tmpvec1[2];
		exp6[3][3] = 1.0;
	}
	else
	{
		vec3_mult_value(v, theta, tmpvec1);
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				exp6[i][j] = I33[i][j];
		exp6[0][3] =  tmpvec1[0];
		exp6[1][3] =  tmpvec1[1];
		exp6[2][3] =  tmpvec1[2];
		exp6[3][3] = 1.0;
	}
	return;
}



/**
*@brief 计算计算齐次变换对应的 6 x 6 伴随变换 [AdT ]
*@param[in]		T		齐次变换矩阵.
*@param[out]	AdT		6x6伴随映射矩阵 [AdT ].
*@return		无返回值.
*@note:
*@warning:
*/
void Adjoint(float_def T[4][4], float_def AdT[6][6])
{
	float_def R[3][3];
	float_def p[3];
	float_def so3Mat[3][3];
	int i;
	int j;
	trans_rp(T, R, p);
	vec_so3(p, so3Mat);

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			AdT[i][j] = R[i][j];
		}
	}
	for (i = 0; i < 3; i++)
	{
		for (j = 3; j < 6; j++)
		{
			AdT[i][j] = 0.0;
		}
	}
	for (i = 3; i < 6; i++)
	{
		for (j = 3; j < 6; j++)
		{
			AdT[i][j] = R[i - 3][j - 3];
		}
	}
	float_def mat[3][3];
	matrix3_mult(so3Mat, R, mat);
	for (i = 3; i < 6; i++)
	{
		for (j = 0; j < 3; j++)
		{
			AdT[i][j] = mat[i - 3][j];
		}
	}

	return;
}

/**
*@brief Description: Computes the body Jacobian Jb(theta) in 6×n given a list of joint screws Bi
expressed in the body frame and a list of joint angles.
*@param[in]		Blist		The joint screw axes in the end - effector frame when the manipulator is
*							at the home position, in the format of a matrix with the screw axes as the row.
*@param[in]		thetalist	A list of joint coordinates.
*@param[out]	Jb			Body Jacobian matrix.
*@return        No return value.
*@note:			 when Blist and Jb are matrixes ,make sure that columns number of Slist or Jb is equal to JointNum,
*				 rows number of Slist or Jb is 6 .The function call should be written as
*				 JacobianSpace(JointNum,(float_def *)Slist,thetalist,(float_def *)Jb).
*@warning:
*/
void jacobian_body(int JointNum, float_def Blist[6][6], float_def thetalist[6], float_def  Jb[6][6])
{
	int i;
	int j;
	int k;
	float_def T1[4][4];
	float_def T2[4][4];
	float_def se3mat[4][4];
	float_def V[6];
	float_def AdT[6][6];
	float_def T[4][4] = {
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
	};

	//Fist column of Jbn.
	for (i = 0; i < 6; i++)
	{
		Jb[i][JointNum-1] = Blist[i][JointNum-1];
	}
	//Jbi for i=n-1,n-2,...,1.
	for (i = JointNum - 2; i >= 0; i--)
	{
		for (j = 0; j < 6; j++)
		{
			V[j] = -1.0 * Blist[j][i+1];
		}
		twist_se3(V, se3mat);
		matrix4_mult_value(se3mat, thetalist[i + 1], se3mat);
		matrix_exp6(se3mat, T1);
		matrix4_mult(T, T1, T2);
		matrix4_cpy(T2, T);
		Adjoint(T, AdT);
		for (j = 0; j < 6; j++)
		{
			Jb[j][i] = 0;
			for (k = 0; k < 6; k++)
			{
				Jb[j][i] = Jb[j][i] + AdT[j][k] * Blist[k][i];
			}
		}
	}
	return;
}

/**
*@brief			Description:Computes the space Jacobian Js(theta) in R6 x n given a list of joint screws Si
*				expressed in the fixed space frame and a list of joint angles.
*@param[in]		JointNum	joints number.
*@param[in]		Slist		The joint screw axes expressed in the fixed space frame when the manipulator is
*							at the home position, in the format of a matrix with the screw axes as the column.
*@param[in]		thetalist	A list of joint coordinates.
*@param[out]	Js			Space Jacobian matrix.
*@return		 No return value.
*@note:			 when Slist and Js are matrices ,make sure that columns number of Slist or Js is equal to JointNum,
*				 rows number of Slist or Js is 6 .The function call should be written as
*				 JacobianSpace(JointNum,(float_def *)Slist,thetalist,(float_def *)Js).
*@warning:
*/
void jacobian_space(int JointNum, float_def Slist[6][6], float_def* thetalist, float_def Js[6][6])
{
	int i;
	int j;
	int k;
	float_def T1[4][4];
	float_def T2[4][4];
	float_def se3mat[4][4];
	float_def V[6];
	float_def AdT[6][6];
	float_def T[4][4] = {
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
	};
	//Fist column of Js.
	for (i = 0; i < 6; i++)
	{
		Js[i][0] = Slist[i][0];
	}
	//Jsi for i=2,3,...,n.
	for (i = 1; i < JointNum; i++)
	{
		for (j = 0; j < 6; j++)
		{
			V[j] = Slist[j][i-1];
		}
		twist_se3(V, se3mat);
		matrix4_mult_value(se3mat, thetalist[i - 1], se3mat);
		matrix_exp6(se3mat, T1);
		matrix4_mult(T, T1, T2);
		matrix4_cpy(T2, T);
		Adjoint(T, AdT);
		for (j = 0; j < 6; j++)
		{
			Js[j][i] = 0;
			for (k = 0; k < 6; k++)
			{
				Js[j][i] = Js[j][i] + AdT[j][k] * Slist[k][i];
			}
		}
	}
	return;
}



/**
 * @brief 			Description: Computes the unit vector of Euler axis and rotation angle corresponding to rotation matrix.
 * @param[in]		R				A rotation matrix.
 * @param[out]		omghat			the unit vector of Euler axis .
 * @param[out]		theta			the rotation angle.
 * @return			No return value.
 * @retval			0
 * @note:			if  theta is zero ,the unit axis is undefined and set it as a zero vector [0;0;0].
 *@warning:
*/
void RotToAxisAng(float_def R[3][3], float_def omghat[3], float_def* theta)
{
	float_def tmp;
	float_def omg[3] = { 0 };
	float_def acosinput = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0;
	if (fabs(acosinput - 1.0) < 1.0E-6)
	{
		memset(omghat, 0, 3 * sizeof(float_def));
		*theta = 0.0;
	}
	else if (acosinput <= -1.0)
	{
		if ((1.0 + R[2][2]) >= 1.0E-6)
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[0][2];
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[1][2];
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * (1.0 + R[2][2]);
		}
		else if ((1.0 + R[1][1] >= 1.0E-6))
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[0][1];
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * (1.0 + R[1][1]);
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[2][1];
		}
		else
		{
			omg[0] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * (1.0 + R[0][0]);
			omg[1] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[1][0];
			omg[2] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[2][0];
		}
		omghat[0] = omg[0];
		omghat[1] = omg[1];
		omghat[2] = omg[2];
		*theta = PM_PI;
	}
	else
	{
		*theta = acos(acosinput);
		tmp = 2.0 * sin(*theta);
		omghat[0] = (R[2][1] - R[1][2]) / tmp;
		omghat[1] = (R[0][2] - R[2][0]) / tmp;
		omghat[2] = (R[1][0] - R[0][1]) / tmp;

	}

	return;
}




///////////////////////////////////////////////////////////////////////////////

/**
 * \brief 求解paden_kahan子问题1，参考《机器人操作的数学导论实现》.
 * 
 * \param w	旋转轴的单位方向向量.
 * \param u	点（或向量）旋转前的坐标.
 * \param v	点（或向量）旋转后的坐标.
 * \return  待求解的旋转角，单位:弧度.
 */
float_def paden_kahan1(float_def w[3], float_def u[3], float_def v[3])
{
	float_def u1[3];
	float_def v1[3];
	float_def m1[3][3];
	float_def vec1[3];
	float_def x, y;
	//u1=u-w*w'*u
	matrix_mult(w, 3, 1, w, 3, (float_def *)m1);
	matrix3_mult_vec(m1, u, vec1);
	vec3_sub(u, vec1, u1);
	//v1=v-w*w'*v
	matrix_mult(w, 3, 1, w, 3, (float_def*)m1);
	matrix3_mult_vec(m1, v, vec1);
	vec3_sub(v, vec1, v1);
	//x=w'*cross(u1,v1)
	vec3_cross(u1, v1, vec1);
	x = w[0] * vec1[0] + w[1] * vec1[1] + w[2] * vec1[2];
	//y=u1'*v1
	y = u1[0] * v1[0] + u1[1] * v1[1] + u1[2] * v1[2];
	return atan2(x, y);
}

/**
 * \brief 求解paden_kahan子问题2，参考《机器人操作的数学导论》实现.
 * 
 * \param w2	第一个旋转轴的方向矢量。
 * \param w2	第二个旋转轴的方向矢量。
 * \param u		点（或向量）旋转前的坐标。
 * \param v		点（或向量）旋转后的坐标。
 * \param theta	返回的两组解，theta[0][0],theta[0][1]构成一组解
 *				theta[1][0],theta[1][1]构成第二组解。
 * \return		返回0执行成功，返回非0执行失败。
 * \retval 1	参数错误，两个轴的方向向量平行。
 * \retval 2	无解，所给参数不存在解。
 * \atention	两个轴的方向矢量不能平行，平行时会返回错误参数1。	
 */
int paden_kahan2(float_def w2[3], float_def w1[3], float_def u[3], float_def v[3],float_def theta[2][2])
{
	int i, j;
	float_def alpha=0;
	float_def beta=0;
	float_def tmp1=0;
	float_def tmp2=0;
	float_def num = 0;
	float_def den = 0;
	float_def vec1[3] = { 0 };
	float_def gama[2] = { 0 };
	float_def c[3] = { 0 };

	//alpha = ((w1'*w2)*w2' * u - w1'*v)/((w1' * w2) ^ 2 - 1);
	tmp1 = w1[0] * w2[0] + w1[1] * w2[1] + w1[2] * w2[2];
	tmp2 = w2[0] * u[0] + w2[1] * u[1] + w2[2] * u[2];
	tmp1 = tmp1 * tmp2;
	tmp2= w1[0] * v[0] + w1[1] * v[1] + w1[2] * v[2];
	num = tmp1 - tmp2;
	
	tmp1= w1[0] * w2[0] + w1[1] * w2[1] + w1[2] * w2[2];
	den = tmp1 * tmp1 - 1.0;
	if (fabs(den) < 1.0e-5)return 1;
	alpha = num / den;

	//beta = ((w1'*w2)*w1' * v - w2'*u)/((w1' * w2) ^ 2 - 1);
	tmp1 = w1[0] * w2[0] + w1[1] * w2[1] + w1[2] * w2[2];
	tmp2 = w1[0] * v[0] + w1[1] * v[1] + w1[2] * v[2];
	tmp1 = tmp1 * tmp2;
	tmp2 = w2[0] * u[0] + w2[1] * u[1] + w2[2] * u[2];
	num = tmp1 - tmp2;
	beta = num / den;

	//gama=((norm(u))^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2))^2);
	vec3_cross(w1, w2,vec1);
	den = vec_norm(3, vec1);
	den = den * den;
	if (den < 1.0E-8)return 1;

	tmp1 = vec_norm(3,u);
	tmp2 = w1[0] * w2[0] + w1[1] * w2[1] + w1[2] * w2[2];
	gama[0] = (tmp1 * tmp1 - alpha * alpha - beta * beta - 2 * alpha * beta * tmp2) / den;
	if (gama[0] < 1.0e-8)return 2;
	gama[0] = sqrt(gama[0]);
	gama[1] = -gama[0];
	theta[0][0] = 0.0; theta[0][1] = 0.0;
	theta[1][0] = 0.0; theta[1][1] = 0.0;
	//计算两组解
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 3; j++)
		{
			c[j] = alpha * w1[j] + beta * w2[j] + gama[i] * vec1[j];
		}
		theta[i][0] = paden_kahan1(w2, u, c);
		theta[i][1] = paden_kahan1(w1, c, v);
	}
	return 0;
}


/**
 * \brief 高斯消去法解线性方程组（注意，没有选列主元）.
 * 
 * \return 1	出现对角线元素为0,方程无法求解
 */
int gauss_solve_linear_equation(float_def *A,int NUM,float_def *b,float_def *X)
{

	int i, j, k;
	int n = NUM;
	float_def mik, S;
	//消元
	for (k = 0; k < NUM - 1; k++)
	{
		if (fabs(A[k*n+k])<1.0E-6)
			return 1;
		for (i = k + 1; i < NUM; i++)
		{
			mik = A[i*n+k] / A[n*k+k];
			for (j = k; j < NUM; j++)
			{
				A[i*n+j] = A[i*n+j] - mik * A[k*n+j];
			}
			b[i] = b[i] - mik * b[k];
		}
	}
	X[NUM - 1] = b[NUM - 1] / A[(NUM - 1)*n+NUM - 1];
	for (k = NUM - 2; k >= 0; k--)
	{
		S = b[k];
		for (j = k + 1; j < NUM; j++)
		{
			S = S - A[k*n+j] * X[j];
		}
		X[k] = S / A[k*n+k];
	}
	return 0;
}

#if 1
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)

/**
*@brief			计算 (a^2+b^2)^(1/2),
*@param[in]		a
*@param[in]		b
*@note:
*@warning:
*/
float_def pythag(const float_def a, const float_def b)
{
	float_def absa, absb;
	absa = fabs(a);
	absb = fabs(b);
	if (absa > absb) return absa * sqrt(1.0 + (absb / absa) * (absb / absa));
	else return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + (absa / absb) * (absa / absb)));

}

/**
 * \brief		Given the output of decompose, this routine sorts the singular values, and corresponding columns
				of u and v, by decreasing magnitude.Also, signs of corresponding columns are flipped so as to
				maximize the number of positive elements.
 * 
 * \param u		
 * \param m
 * \param n
 * \param w
 * \param v
 */
void reorder(float_def* u, int m, int n, float_def* w, float_def* v)
{
	int i, j, k, s, inc = 1;
	float_def sw;
	float_def su[PINV_MAX];
	float_def sv[PINV_MAX];
	do { inc *= 3; inc++; } while (inc <= n);// Sort.The method is Shell’s sort.(The work is negligible as compared to that already done in decompose.)
	do {
		inc /= 3;
		for (i = inc; i < n; i++) {
			sw = w[i];
			for (k = 0; k < m; k++) su[k] = u[k * n + i];
			for (k = 0; k < n; k++) sv[k] = v[k * n + i];
			j = i;
			while (w[j - inc] < sw) {
				w[j] = w[j - inc];
				for (k = 0; k < m; k++) u[k * n + j] = u[k * n + j - inc];
				for (k = 0; k < n; k++) v[k * n + j] = v[k * n + j - inc];
				j -= inc;
				if (j < inc) break;
			}
			w[j] = sw;
			for (k = 0; k < m; k++) u[k * n + j] = su[k];
			for (k = 0; k < n; k++) v[k * n + j] = sv[k];
		}
	} while (inc > 1);
	for (k = 0; k < n; k++) {
		//Flip signs.
		s = 0;
		for (i = 0; i < m; i++) if (u[i * n + k] < 0.0) s++;
		for (j = 0; j < n; j++) if (v[j * n + k] < 0.0) s++;
		if (s > (m + n) / 2) {
			for (i = 0; i < m; i++) u[i * n + k] = -u[i * n + k];
				for (j = 0; j < n; j++) v[j * n + k] = -v[j * n + k];
		}
	}
	return;
}


/**
 * \brief		对任意阶矩阵进行svd分解，注意输入的矩阵a是通过二维数组定义的维数固定的矩阵,不能使用dmatrix创建的矩阵。
 *				参考[C数值算法]William.H.Press。
 * 
 * \param a		输入/输出矩阵a,svd分解的结果u存放在矩阵a中，覆盖原矩阵。
 * \param m		矩阵的行数。
 * \param n		矩阵的列数。
 * \param tol	奇异值的误差值。
 * \param w		分解的向量。
 * \param v		分解的矩阵v,注意需要把二维名转为一维指针。
 * \return		返回值0执行成功，非零执行失败
 * \retval 1	矩阵维数超过了允许值。
 */
int svdcmp(float_def* a, int m, int n, float_def tol, float_def* w, float_def* v)
{

	int flag=0, i=0, its=0, j=0, jj=0, k=0, l=0, nm=0;
	float_def anorm, c, f, g, h, s, scale, x, y, z;
	float_def rv1[PINV_MAX] = { 0 };
	if (m > PINV_MAX || n > PINV_MAX)return 1;
	g = scale = anorm = 0.0;
	for (i = 0; i < n; i++)//Householder reduction to bidiagonal form.
	{
		l = i + 2;
		rv1[i] = scale * g;
		g = s = scale = 0.0;
		if (i < m)
		{
			for (k = i; k < m; k++)scale += fabs(a[k * n + i]);
			if (scale != 0.0)
			{
				for (k = i; k < m; k++)
				{
					a[k * n + i] /= scale;
					s += a[k * n + i] * a[k * n + i];
				}
				f = a[i * n + i];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i * n + i] = f - g;
				for (j = l - 1; j < n; j++)
				{
					for (s = 0.0, k = i; k < m; k++)
					{
						s += a[k * n + i] * a[k * n + j];
					}
					f = s / h;
					for (k = i; k < m; k++)
					{
						a[k * n + j] += f * a[k * n + i];
					}
				}
				for (k = i; k < m; k++)
				{
					a[k * n + i] *= scale;
				}
			}
		}
		w[i] = scale * g;
		g = s = scale = 0.0;
		if (i + 1 <= m && (i + 1) != n)
		{
			for (k = l - 1; k < n; k++)
			{
				scale += fabs(a[i * n + k]);
			}
			if (scale != 0.0)
			{
				for (k = l - 1; k < n; k++)
				{
					a[i * n + k] /= scale;
					s += a[i * n + k] * a[i * n + k];
				}
				f = a[i * n + l - 1];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i * n + l - 1] = f - g;
				for (k = l - 1; k < n; k++)
				{
					rv1[k] = a[i * n + k] / h;
				}
				for (j = l - 1; j < m; j++)
				{
					for (s = 0.0, k = l - 1; k < n; k++)
					{
						s += a[j * n + k] * a[i * n + k];
					}
					for (k = l - 1; k < n; k++)
					{
						a[j * n + k] += s * rv1[k];
					}
				}
				for (k = l - 1; k < n; k++)
				{
					a[i * n + k] *= scale;
				}
			}
		}
		anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}

	for (i = n - 1; i >= 0; i--)//Accumulation of right-hand transformations.
	{
		if (i < n - 1)
		{
			if (g != 0.0)
			{
				for (j = l; j < n; j++)//Double division to avoid possible underflow.
				{
					v[j * n + i] = (a[i * n + j] / a[i * n + l]) / g;
				}
				for (j = l; j < n; j++)
				{
					for (s = 0.0, k = l; k < n; k++)
					{
						s += a[i * n + k] * v[k * n + j];
					}
					for (k = l; k < n; k++)
					{
						v[k * n + j] += s * v[k * n + i];
					}
				}
			}
			for (j = l; j < n; j++)
			{
				v[i * n + j] = v[j * n + i] = 0.0;
			}
		}
		v[i * n + i] = 1.0;
		g = rv1[i];
		l = i;
	}

	for (i = MIN(m, n) - 1; i >= 0; i--)//Accumulation of left-hand transformations.
	{
		l = i + 1;
		g = w[i];
		for (j = l; j < n; j++)
		{
			a[i * n + j] = 0.0;
		}
		if (g != 0.0)
		{
			g = 1.0 / g;
			for (j = l; j < n; j++)
			{
				for (s = 0.0, k = l; k < m; k++)
				{
					s += a[k * n + i] * a[k * n + j];
				}
				f = (s / a[i * n + i]) * g;
				for (k = i; k < m; k++)
				{
					a[k * n + j] += f * a[k * n + i];
				}
			}
			for (j = i; j < m; j++)
			{
				a[j * n + i] *= g;
			}
		}
		else
		{
			for (j = i; j < m; j++)
			{
				a[j * n + i] = 0.0;
			}
		}
		++a[i * n + i];
	}

	for (k = n - 1; k >= 0; k--)
	{
		/* Diagonalization of the bidiagonal form: Loop over
		singular values, and over allowed iterations. */
		for (its = 0; its < 30; its++)
		{
			flag = 1;
			for (l = k; l >= 0; l--)//Test for splitting.
			{
				nm = l - 1;
				if (l == 0 || fabs(rv1[l]) <= tol * anorm)
				{
					flag = 0;
					break;
				}
				if (fabs(w[nm]) <= tol * anorm)break;

			}
			if (flag)//Cancellation of rv1[l], if l > 0.
			{
				c = 0.0;
				s = 1.0;
				for (i = l; i < k + 1; i++)
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if (fabs(f) <= tol * anorm)break;
					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for (j = 0; j < m; j++)
					{
						y = a[j * n + nm];
						z = a[j * n + i];
						a[j * n + nm] = y * c + z * s;
						a[j * n + i] = z * c - y * s;
					}
				}
			}
			z = w[k];
			if (l == k)//Convergence
			{
				if (z < 0.0)//Singular value is made nonnegative..
				{
					w[k] = -z;
					for (j = 0; j < n; j++)
					{
						v[j * n + k] = -v[j * n + k];
					}
				}
				break;
			}
			x = w[l]; //Shift from bottom 2-by-2 minor.
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
			g = pythag(f, 1.0);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0;
			for (j = l; j <= nm; j++)//Next QR transformation:
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;
				for (jj = 0; jj < n; jj++)
				{
					x = v[jj * n + j];
					z = v[jj * n + i];
					v[jj * n + j] = x * c + z * s;
					v[jj * n + i] = z * c - x * s;
				}
				z = pythag(f, h);
				w[j] = z;
				if (z)//Rotation can be arbitrary if z = 0.
				{
					z = 1.0 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;
				for (jj = 0; jj < m; jj++)
				{
					y = a[jj * n + j];
					z = a[jj * n + i];
					a[jj * n + j] = y * c + z * s;
					a[jj * n + i] = z * c - y * s;
				}
			}
			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
	reorder(a, m, n, w, v);//对特征值按降序排列，且调整对应矩阵
	return 0;
}


/**
 * \brief		使用奇异值分解 (SVD) 算法计算任意阶矩阵的伪逆（Moore-Penrose pseudoinverse）。
 * 
 * \param a		矩阵a。
 * \param m		矩阵a的行数，最大值由MathLib.h的宏PINV_MAX决定。
 * \param n		矩阵a的列数，最大值由MathLib.h的宏PINV_MAX决定。
 * \param tol	奇异值得的误差值。
 * \param b		计算结果--伪逆。
 * \return		返回值0执行成功，非0执行失败。
 * \retval 1    矩阵的维数超出限制，最大值由MathLib.h的宏PINV_MAX决定。
 */
int matrix_pinv(const float_def* a, int m, int n, float_def tol, float_def* b)
{
	int i, j;
	int ret;
	static float_def u[PINV_MAX * PINV_MAX] = { 0 };
	static float_def w[PINV_MAX] = { 0 };
	static float_def vw[PINV_MAX * PINV_MAX] = { 0 };
	static float_def v[PINV_MAX * PINV_MAX] = { 0 };
	static float_def uT[PINV_MAX * PINV_MAX] = { 0 };
	if (m > PINV_MAX || n > PINV_MAX)return 1;
	memcpy(u, a, m * n * sizeof(float_def));
	ret = svdcmp(u, m, n, tol, w, v);
	if (ret)return ret;
	for (i = 0; i < n; i++)
	{
		if (w[i] <= tol)
		{
			w[i] = 0;
		}
		else
		{
			w[i] = 1.0 / w[i];
		}
	}
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			vw[i * n + j] = v[i * n + j] * w[j];
		}
	}
	matrix_tranpose(u, m, n, uT);
	matrix_mult(vw, n, n, uT, m, b);
	return 0;
} 
#endif

/**
 * \brief 六维力传感器后方工具质心校准
 * \param 输入forceIni:6-14组静态试验下的六维力传感器读数，每一行为一组数据，，国际单位制
 * \param 输出centroid：前3个元素为传感器坐标系的工具质心位置。
*/
int toolCentroidCalibrate(double forceIni[6][6], double centroid[6])//工具质心校准函数，输入多组静态试验的原始数据
{
	//int m = 6;//6-14组试验，至少为3组试验，但要满足试验姿态不共面
	double M[3 * 6][1] = { 0 };//力矩列向量，第1-3、4-6元素分别为第一组、第二组MX,MY,MZ力矩读数
	double f[3 * 6][6] = { 0 };
	double f_pinv[6][3 * 6] = { 0 };
	for (int i = 0; i < 3 * 6; i++)
	{
		int num = 1 + (int)floor(i / 3);//调用第num组试验数据赋值
		int num2 = 1 + (int)fmod(i, 3);//第mum中的第1-3个维度
		M[i][0] = forceIni[num - 1][num2 - 1 + 3];
		if (num2 == 1)
		{
			f[i][1] = forceIni[num - 1][2];
			f[i][2] = -forceIni[num - 1][1];
			f[i][3] = 1;
		}
		if (num2 == 2)
		{
			f[i][0] = -forceIni[num - 1][2];
			f[i][2] = forceIni[num - 1][0];
			f[i][4] = 1;
		}
		if (num2 == 3)
		{
			f[i][0] = forceIni[num - 1][1];
			f[i][1] = -forceIni[num - 1][0];
			f[i][5] = 1;
		}
	}
	matrix_pinv(*f, 3 * 6, 6, 0.0001, *f_pinv);
	matrix_mult(*f_pinv, 6, 3 * 6, *M, 1, centroid);
	return 0;
}

/**
 * \brief 六维力传感器零点和后方工具重力辨识
 * \param 输入1：forceIni:n组(n=6-14)静态试验下的六维力传感器读数，每一行为一组数据，，国际单位制
 * \param 输入2：rpy：静态试验时，工具坐标系相对机器人基坐标系的姿态角(固定参考系，次序为XYZ轴),n*3
 * \param 输入3：rpySensor：传感器坐标系下，工具坐标系的姿态角rpy
 * \param 输出1:return工具重力大小
 * \param 输出2:SensorZero传感器零点(本次试验下的零点数值)
*/
double toolGravityAndSensorZero(double forceIni[6][6], double rpy[6][3], double rpySensor[3], double centroid[6], double SensorZero[6])//工具质心校准函数，输入多组静态试验的原始数据
{
	//int m = 6;//7-14组试验，至少为3组试验，但要满足试验姿态不共面
	double f_trans[3 * 6][1] = { 0 };//平移力列向量，第1-3、4-6元素分别为第一组、第二组fx,fy,fz力读数
	double R_stci[3 * 6][6] = { 0 };//姿态分块矩阵
	//double gravitySensor[6] = { 0 };
	double temr1[3][3] = { 0 };
	double temr2[3][3] = { 0 };
	double temr3[3][3] = { 0 };
	rotx(rpySensor[0] * 1, temr1);
	roty(rpySensor[1] * 1, temr2);
	rotz(rpySensor[2] * 1, temr3);
	double temr4[3][3] = { 0 };
	matrix_mult(*temr2, 3, 3, *temr1, 3, *temr4);
	double Rst[3][3] = { 0 };//工具坐标系相对传感器坐标系的旋转矩阵，重点1
	matrix_mult(*temr3, 3, 3, *temr4, 3, *Rst);
	double temrx[3][3] = { 0 };
	double temry[3][3] = { 0 };
	double temrz[3][3] = { 0 };
	double temrstc[3][3] = { 0 };
	for (int ii = 0; ii < 3 * 6; ii++)
	{
		int num = 1 + (int)floor(ii / 3);//调用第num组试验数据赋值
		int num2 = 1 + (int)fmod(ii, 3);//第mum中的第1-3个维度
		f_trans[ii][0] = forceIni[num - 1][num2 - 1];
	}
	for (int i = 0; i < 6; i++)
	{
		rotx(rpy[i][0] * 1, temrx);//57.29为弧度制向度数值转化的近似数
		roty(rpy[i][1] * 1, temry);
		rotz(rpy[i][2] * 1, temrz);
		double temr5[3][3] = { 0 };
		matrix_mult(*temry, 3, 3, *temrx, 3, *temr5);
		double Rct[3][3] = { 0 };//机器人基座/工具质心过渡坐标系下，工具坐标系的旋转矩阵
		double Rtc[3][3] = { 0 };
		matrix_mult(*temrz, 3, 3, *temr5, 3, *Rct);
		matrix_tranpose(*Rct, 3, 3, *Rtc);
		matrix_mult(*Rst, 3, 3, *Rtc, 3, *temrstc);
		R_stci[3 * i][0] = temrstc[0][0];
		R_stci[3 * i][1] = temrstc[0][1];
		R_stci[3 * i][2] = temrstc[0][2];
		R_stci[3 * i][3] = 1;

		R_stci[3 * i + 1][0] = temrstc[1][0];
		R_stci[3 * i + 1][1] = temrstc[1][1];
		R_stci[3 * i + 1][2] = temrstc[1][2];
		R_stci[3 * i + 1][4] = 1;

		R_stci[3 * i + 2][0] = temrstc[2][0];
		R_stci[3 * i + 2][1] = temrstc[2][1];
		R_stci[3 * i + 2][2] = temrstc[2][2];
		R_stci[3 * i + 2][5] = 1;
	}
	double R_stci_pinv[6][3 * 6] = { 0 };
	matrix_pinv(*R_stci, 3 * 6, 6, 0.0001, *R_stci_pinv);

	matrix_mult(*R_stci_pinv, 6, 3 * 6, *f_trans, 1, SensorZero);//先得到力零点，在sensorzeros第4-6个元素。
	double gravity = -SensorZero[2];//假设机器人基坐标系的z轴为严格竖直方向，那么SensorZero[0]，SensorZero[1]=0；
	SensorZero[0] = centroid[3] - SensorZero[4] * centroid[2] + SensorZero[5] * centroid[1];
	SensorZero[1] = centroid[4] - SensorZero[5] * centroid[0] + SensorZero[3] * centroid[2];
	SensorZero[2] = centroid[5] - SensorZero[3] * centroid[1] + SensorZero[4] * centroid[0];
	return gravity;
	//return 0;
}

/**
 * \brief 六维力传感器零点通电更新
 * \param 输入1：force:开机通电后机械臂静态试验下的六维力传感器读数（最好是滤波后数据）
 * \param 输入2：rpy：静态试验时，工具坐标系相对机器人基坐标系的姿态角rpy(固定参考系，次序为XYZ轴)
 * \param 输入3：rpySensor：传感器坐标系下，工具坐标系的姿态角rpy
 * \param 输入4:工具重力大小，质心位置（传感器坐标系下数值）
 * \param 输出1:SensorZero传感器零点(本次通电下更新的零点数值)
*/
int sensorZeroUpdata(double force[6], double rpy[3], double rpySensor[3], double gravityAll[4], double sensorZero[6])
{
	double G = gravityAll[0];//工具重力大小
	double center[3] = { gravityAll[1],gravityAll[2],gravityAll[3] };//传感器坐标系下，工具质心位置
	double G_sensor[6] = { 0 };//传感器坐标系下的工具重力负载
	//double G_center[6] = { 0,0,-G,0,0,0 };//重点，质心坐标系下的工具重力负载，该坐标系与机器人坐标系姿态一致，且机器人基坐标系z轴为竖直向上（重力竖直向下）

	double temr1[3][3] = { 0 };
	double temr2[3][3] = { 0 };
	double temr3[3][3] = { 0 };
	rotx(rpySensor[0] * 1, temr1);
	roty(rpySensor[1] * 1, temr2);
	rotz(rpySensor[2] * 1, temr3);
	double temr4[3][3] = { 0 };
	matrix_mult(*temr2, 3, 3, *temr1, 3, *temr4);
	double Rst[3][3] = { 0 };//传感器坐标系下，工具坐标系的的旋转矩阵，重点1
	matrix_mult(*temr3, 3, 3, *temr4, 3, *Rst);

	rotx(rpy[0] * 1, temr1);//57.29为弧度制向度数值转化的近似数
	roty(rpy[1] * 1, temr2);
	rotz(rpy[2] * 1, temr3);
	matrix_mult(*temr2, 3, 3, *temr1, 3, *temr4);
	double Rct[3][3] = { 0 };//机器人基座/工具质心坐标系下，工具坐标系的旋转矩阵
	matrix_mult(*temr3, 3, 3, *temr4, 3, *Rct);
	double Rtc[3][3] = { 0 };
	//matrix_tranpose(double* a, int m, int n, double* b)
	matrix_tranpose(*Rct, 3, 3, *Rtc);
	double Rsc[3][3] = { 0 };//重点，传感器坐标系下，质心坐标系的旋转矩阵
	matrix_mult(*Rst, 3, 3, *Rtc, 3,*Rsc);

	double G_sensor_trans[3][1] = { {0},{0},{-G} };
	double tem1[3][1] = { 0 };
	matrix_mult(*Rsc, 3, 3, *G_sensor_trans, 1, *tem1);
	G_sensor[0] = tem1[0][0];
	G_sensor[1] = tem1[1][0];
	G_sensor[2] = tem1[2][0];

	//生成关于位置向量的伴随矩阵pX,   G_center[6] = { 0,0,-G,0,0,0 }
	double px[3][3] = { {0, -center[2],center[1]},{center[2],0,-center[0]},{-center[1],center[0],0} };
	matrix_mult(*px, 3, 3, *Rsc, 3, *temr1);
	double tem2[3][1] = { 0 };
	matrix_mult(*temr1, 3, 3, *G_sensor_trans, 1, *tem2);
	G_sensor[3] = tem2[0][0];
	G_sensor[4] = tem2[1][0];
	G_sensor[5] = tem2[2][0];

	sensorZero[0] = force[0] - G_sensor[0];//无负载下，传感器零点有原始读数减去重力负载即可
	sensorZero[1] = force[1] - G_sensor[1];
	sensorZero[2] = force[2] - G_sensor[2];
	sensorZero[3] = force[3] - G_sensor[3];
	sensorZero[4] = force[4] - G_sensor[4];
	sensorZero[5] = force[5] - G_sensor[5];
	return 0;
}

/**
 * \brief 计算工具重力负载（传感器坐标系下）
 * \param 输入1：rpy: 基坐标系下，工具姿态rpy，由正向运动学给出
 * \param 输入2：rpySensor： 传感器坐标系下，工具姿态rpy,固定值
 * \param 输入3：gravityAll  工具重力大小，质心位置（传感器坐标系下），提前辨识给出
 * \param 输出1:G_sensor     传感器坐标系中的工具重力负载
*/
int toolGravitySensor(double rpy[3], double rpySensor[3], double gravityAll[4], double G_sensor[6])
{
	double G = gravityAll[0];//工具重力大小
	double center[3] = { gravityAll[1],gravityAll[2],gravityAll[3] };//传感器坐标系下，工具质心位置

	double temr1[3][3] = { 0 };
	double temr2[3][3] = { 0 };
	double temr3[3][3] = { 0 };
	rotx(rpySensor[0] * 1, temr1);
	roty(rpySensor[1] * 1, temr2);
	rotz(rpySensor[2] * 1, temr3);
	double temr4[3][3] = { 0 };
	matrix_mult(*temr2, 3, 3, *temr1, 3, *temr4);
	double Rst[3][3] = { 0 };//传感器坐标系下，工具坐标系的的旋转矩阵，重点1
	matrix_mult(*temr3, 3, 3, *temr4, 3, *Rst);

	rotx(rpy[0] * 1, temr1);//57.29为弧度制向度数值转化的近似数
	roty(rpy[1] * 1, temr2);
	rotz(rpy[2] * 1, temr3);
	matrix_mult(*temr2, 3, 3, *temr1, 3, *temr4);
	double Rct[3][3] = { 0 };//机器人基座/工具质心坐标系下，工具坐标系的旋转矩阵
	matrix_mult(*temr3, 3, 3, *temr4, 3, *Rct);
	double Rtc[3][3] = { 0 };
	//matrix_tranpose(double* a, int m, int n, double* b)
	matrix_tranpose(*Rct, 3, 3, *Rtc);
	double Rsc[3][3] = { 0 };//重点，传感器坐标系下，质心坐标系的旋转矩阵
	matrix_mult(*Rst, 3, 3, *Rtc, 3, *Rsc);

	double G_sensor_trans[3][1] = { {0},{0},{-G} };
	double tem1[3][1] = { 0 };
	matrix_mult(*Rsc, 3, 3, *G_sensor_trans, 1, *tem1);
	G_sensor[0] = tem1[0][0];
	G_sensor[1] = tem1[1][0];
	G_sensor[2] = tem1[2][0];

	//生成关于位置向量的伴随矩阵pX,   G_center[6] = { 0,0,-G,0,0,0 }
	double px[3][3] = { {0, -center[2],center[1]},{center[2],0,-center[0]},{-center[1],center[0],0} };
	matrix_mult(*px, 3, 3, *Rsc, 3, *temr1);
	double tem2[3][1] = { 0 };
	matrix_mult(*temr1, 3, 3, *G_sensor_trans, 1, *tem2);
	G_sensor[3] = tem2[0][0];
	G_sensor[4] = tem2[1][0];
	G_sensor[5] = tem2[2][0];
	return 0;
}


/**
 * \brief 六维力传感器环境有效负载的提取
 *
 * \param force    传感器读数
 * \param sensorZero     传感器零点
 * \param rpy            基坐标系下，工具姿态rpy，由正向运动学给出
 * \param rpySensor      传感器坐标系下，工具姿态rpy,固定值
 * \param gravityAll     工具重力大小，质心位置（传感器坐标系下），提前辨识给出
 * \param load           环境接触力
 */
int contactForceGet(double force[6], double sensorZero[6], double rpy[3], double rpySensor[3], double gravityAll[4],double load[6])
{
	double G_sensor[6] = { 0 }; 
	toolGravitySensor(rpy, rpySensor, gravityAll, G_sensor);
	for (int i = 0; i < 6; i++)
	{
		load[i] = force[i] - sensorZero[i] - G_sensor[i];
	}
	return 0;
}

