/*****************************************************************//**
 * \file   MathLib.h
 * \brief  自定义的数学函数库
 * 
 * \author LiBing
 * \date   June 2020
 */

#ifndef MAHTLIB_H_
#define MAHTLIB_H_

#ifdef __cplusplus
extern "C" {
#endif
#ifndef  float_def 
#define float_def double
#endif
	extern int VerSion;
	#define	 PM_PI    3.141592653589793
	#define	 PINV_MAX	18			/**<求矩阵伪逆的最大阶数*/

	/**
	 * \brief 计算两个坐标的欧拉距离.
	 *	if(len<0.01)return 1;//参考向量为0向量,返回错误。
	 * \param n	坐标的维数
	 * \param pos1	第一个坐标
	 * \param pos2	第二个坐标
	 * \return
	 */
	float_def euler_dist(int n, float_def* pos1, float_def* pos2);

	 /**
	  * \brief 计算n维向量的2范数。
	  *
	  * \param n		向量元素的个数。
	  * \param vec		向量数组。
	  * \return			返回向量的2范数。
	  */
	 float_def vec_norm(int n, float_def* vec);

	 /**
	 * \brief 创建一个float_def类型的n维向量，即使用malloc给n维向量分配内存。
	 *
	 * \param n		向量的维数。
	 * \return		一维数组的指针。
	 */
	 float_def* dvector(int n);

	 /**
	  * \brief		使用malloc给一个矩阵分配内存，返回二维数组的指针，可以用a[i][j]的形式访问每个元素。
	  *
	  * \param row	矩阵的行数。
	  * \param col	矩阵的列数。
	  * \return		返回NULL，则内存分配失败。
	  */
	 float_def** dmatrix(int row, int col);

	 /**
	 * \brief		释放float_def类型的矩阵。
	 *
	 * \param m		指向矩阵的指针。
	 * \param row	矩阵的行数。
	 * \param col	矩阵的列数。
	 */
	 void free_dmatrix(float_def** m);

	 /**
	  * \brief		用于复制dmatrix函数创建的矩阵。矩阵a的元素赋值给矩阵b。
	  *
	  * \param row	矩阵行数。
	  * \param col	矩阵列数。
	  * \param a	源矩阵。
	  * \param b	被赋值的矩阵。
	  * \attention  注意只能用于复制dmatrix函数创建的矩阵。
	  */
	void dmatrix_cpy(int m, int n, float_def** a, float_def** b);

	/**
	 * \brief		用于复制固定维数的数组型矩阵，二维数组指针需转化为一级指针，否则编译会报错。
	 *
	 * \param row	矩阵行数。
	 * \param col	矩阵列数。
	 * \param a		源矩阵。
	 * \param b		被赋值的矩阵。
	 * \attention	注意只能用于复制固定维数的数组型矩阵。
	 */
	void matrix_cpy(int m, int n, float_def* a, float_def* b);


	/**
	 * \brief		3维向量复制。
	 *
	 * \param vec1	3维向量。
	 * \param vec2	3维向量。
	 */
	void vec3_cpy(float_def vec1[3], float_def vec2[3]);

	/**
	 * \brief		三阶数组矩阵的复制.
	 *
	 * \param a		源矩阵。
	 * \param b		被赋值的矩阵。
	 * \attention   存储矩阵的内存块必须连续，适用于固定维数的矩阵。
	 */
	void matrix3_cpy(float_def a[][3], float_def b[][3]);

	/**
	 * \brief		四阶数组型矩阵复制。
	 *
	 * \param a		源矩阵。
	 * \param b		被赋值的矩阵。
	 * \attention   存储矩阵的内存块必须连续，适用于固定维数的矩阵或malloc分配内存的矩阵。
	 */
	void matrix4_cpy(float_def a[][4], float_def b[][4]);


	/**
	 * \bief	3阶矩阵相加。
	 *
	 * \param a	三阶矩阵。
	 * \param b	三阶矩阵。
	 * \param c	结果，三阶矩阵。
	 */
	void matrix3_add(float_def a[][3], float_def b[][3], float_def c[][3]);

	/**
	 * \brief		四阶矩阵加法。
	 *
	 * \param a		矩阵a。
	 * \param b		矩阵b。
	 * \param c		矩阵c。
	 */
	void matrix4_add(float_def a[][4], float_def b[][4], float_def c[][4]);

	/**
	 * \brief		矩阵减法。
	 *
	 * \param a		矩阵a。
	 * \param b		矩阵b。
	 * \param c		矩阵c。
	 */
	void matrix3_sub(float_def a[][3], float_def b[][3], float_def c[][3]);

	/**
	 * \brief		四阶矩阵减法。
	 *
	 * \param a		矩阵a。
	 * \param b		矩阵b。
	 * \param c		矩阵c。
	 */
	void matrix4_sub(float_def a[][4], float_def b[][4], float_def c[][4]);

	/**
	 * \brief	3阶矩阵乘法。
	 *
	 * \param a		矩阵a。
	 * \param b		矩阵b。
	 * \param c		矩阵c。
	 */
	void matrix3_mult(float_def a[][3], float_def b[][3], float_def c[][3]);

	/**
	 * \brief		4阶矩阵乘法.
	 *
	 * \param a		矩阵a。
	 * \param b		矩阵b。
	 * \param c		矩阵c。
	 */
	void matrix4_mult(float_def a[][4], float_def b[][4], float_def c[][4]);


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
	void dmatrix_mult( float_def** a, int m, int n, const float_def** b, int l, float_def** c);


	/**
	 * \brief		任意阶矩阵乘法，适用于固定维数的数组型矩阵乘法（或者malloc分配的内存连续的矩阵）。
	 *
	 * \param a		矩阵a。
	 * \param m		矩阵a的行数。
	 * \param n		矩阵a的列数。
	 * \param b		矩阵b。
	 * \param l		矩阵b的列数。
	 * \param c		矩阵a*b的结果。
	 */
	void matrix_mult( float_def* a, int m, int n,const float_def* b, int l, float_def* c);



	/**
	 * \brief		3阶矩阵乘以一个数值。
	 *
	 * \param a		矩阵a。
	 * \param Value	数值。
	 * \param c		矩阵c。
	 */
	void matrix3_mult_value(float_def a[][3], float_def Value, float_def c[][3]);

	/**
	 * \brief		4阶矩阵乘以一个数值。
	 *
	 * \param a		矩阵a。
	 * \param Value	数值。
	 * \param c		矩阵c。
	 */
	void matrix4_mult_value(float_def a[][4], float_def Value, float_def c[][4]);

	/**
	 * \brief		3阶矩阵乘以一个向量。
	 *
	 * \param R		3阶矩阵。
	 * \param vec1	3维向量。
	 * \param vec2	3维向量。
	 */
	void matrix3_mult_vec(float_def R[3][3], float_def vec1[3], float_def vec2[3]);

	/**
	 * \brief		4阶矩阵乘以一个向量。
	 *
	 * \param R		4阶矩阵。
	 * \param vec1	4维向量。
	 * \param vec2	4维向量。
	 */
	void matrix4_mult_vec(float_def T[4][4], float_def vec1[4], float_def vec2[4]);

	/**
	 * \brief		3维向量相加。
	 *
	 * \param vec1	3维向量。
	 * \param vec2	3维向量。
	 * \param vec3	3维向量。
	 */
	void vec3_add(float_def vec1[3], float_def vec2[3], float_def vec3[3]);

	/**
	 * \brief		3维向量叉乘。
	 *
	 * \param vec1	3维向量。
	 * \param vec2	3维向量。
	 * \param vec3	3维向量。
	 */
	void vec3_cross(float_def vec1[3], float_def vec2[3], float_def vec3[3]);

	/**
	 * \brief		3维向量乘以一个数值。
	 *
	 * \param vec1
	 * \param value
	 * \param vec2
	 */
	void vec3_mult_value(float_def vec1[3], float_def value, float_def vec2[3]);

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
	void vec3_twist(int type, float_def vec[3], float_def q[3], float_def twist[6]);


	/**
	 * \brief		计算矩阵的转置，适用于固定维数的数组型矩阵（或内存连续的矩阵）。
	 *
	 * \param a		矩阵a。
	 * \param m		矩阵的行数。
	 * \param n		矩阵的列数。
	 * \param b		转置后的矩阵。
	 */
	void matrix_tranpose(float_def* a, int m, int n, float_def* b);


	/**
	 * \brief	计算两个向量的夹角，范围[0,pi]，单位弧度.
	 *
	 * \param dim	向量的维度。
	 * \param v1	向量1。
	 * \param v2	向量2。
	 * \return		两个向量的夹角，范围[0,pi],单位弧度
	 */
	float_def calc_vector_angle(int dim, float_def v1[3], float_def v2[3]);


	/**
	 * \brief	3阶矩阵求逆（仅适用于旋转矩阵求逆）.
	 *
	 * \param R	旋转矩阵。
	 * \param invR	旋转矩阵的逆。
	 */
	void rot_inv(float_def R[3][3], float_def invR[3][3]);

	/**
	 * \brief	旋转矩阵和位移构建齐次变换矩阵。
	 *
	 * \param R	旋转矩阵。
	 * \param p	位移。
	 * \param T	齐次变换矩阵。
	 */
	void rp_trans(float_def R[3][3], float_def p[3], float_def T[4][4]);

	/**
	 * \brief	从齐次变换矩阵中提取旋转矩阵和位移分量。
	 *
	 * \param T	齐次变换矩阵。
	 * \param R	旋转矩阵。
	 * \param p	位移分量。
	 */
	void trans_rp(float_def T[4][4], float_def R[3][3], float_def p[3]);

	/**
	 * \brief	计算齐次变换矩阵的逆矩阵。
	 *
	 * \param T		齐次变换矩阵。
	 * \param InvT	齐次变换矩阵对应的逆矩阵。
	 */
	void trans_inv(float_def T[4][4], float_def InvT[4][4]);

	 /**
	  * \brief		rpy角转化为旋转矩阵。
	  *
	  * \param rpy	绕固定坐标系的roll,pitch,yaw角度，单位：弧度。
	  * \param R	旋转矩阵。
	  */
	void rpy2r(float_def rpy[3], float_def R[3][3]);	

	/**
	 * \brief		旋转矩阵计算rpy角。
	 *
	 * \param R		旋转矩阵。
	 * \param rpy	rpy角,单位：弧度。
	 */
	void r2rpy(float_def R[3][3], float_def rpy[3]);

	/**
	 * \brief 四元数转化为旋转矩阵。
	 *
	 * \note 四元数分量顺序固定为 [qx, qy, qz, w]。
	 * \note 输入 q 应为单位四元数；若不是单位四元数，请先归一化再调用。
	 * \param q		四元数 [qx, qy, qz, w]。
	 * \param R		旋转矩阵。
	*/
	void q2rot(float_def q[4], float_def R[3][3]);

	/**
	 *
	 * \brief 旋转矩阵转化为四元数。
	 *
	 * \param R		旋转矩阵。
	 * \param q		输出四元数 [qx, qy, qz, w]。
	 * \note 输出四元数的分量顺序固定为 [qx, qy, qz, w]，并满足单位化要求。
	*/
	void rot2q(float_def R[3][3], float_def q[4]);

	/**
	 * \brief 四元数转化为旋转矩阵。
	 *
	 * \note 四元数分量顺序固定为 [w, x, y, z]。
	 * \note 输入 quat 若非单位四元数，函数内部会先归一化。
	 * \param quat	输入四元数 [w, x, y, z]。
	 * \param R		旋转矩阵。
	 */
	void quat2rot(float_def quat[4], float_def R[3][3]);

	/**
	 * \brief 旋转矩阵转化为四元数。
	 *
	 * \note 输出四元数分量顺序固定为 [w, x, y, z]，并满足单位化要求。
	 * \param R		旋转矩阵。
	 * \param quat	输出四元数 [w, x, y, z]。
	 */
	void rot2quat(float_def R[3][3], float_def quat[4]);

	/**
	 * \brief rpy角转化为四元数。
	 *
	 * \note 输出四元数分量顺序固定为 [w, x, y, z]，并满足单位化要求。
	 * \note 角度约定与 rpy2r 一致：固定坐标系 XYZ（roll,pitch,yaw）等价于动坐标系 ZYX（yaw,pitch,roll）。
	 * \note 等价旋转关系：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
	 * \note 对应单位四元数（[w,x,y,z]）公式：
	 *       w = cr*cp*cy + sr*sp*sy
	 *       x = sr*cp*cy - cr*sp*sy
	 *       y = cr*sp*cy + sr*cp*sy
	 *       z = cr*cp*sy - sr*sp*cy
	 *       其中 cr=cos(roll/2), sr=sin(roll/2), cp=cos(pitch/2), sp=sin(pitch/2), cy=cos(yaw/2), sy=sin(yaw/2)。
	 * \param rpy	roll,pitch,yaw（弧度）。
	 * \param quat	输出四元数 [w, x, y, z]。
	 */
	void rpy2quat(float_def rpy[3], float_def quat[4]);

	/**
	 * \brief 四元数转化为rpy角。
	 *
	 * \note 四元数分量顺序固定为 [w, x, y, z]。
	 * \note 输入 quat 若非单位四元数，函数内部会先归一化。
	 * \note 输出角度约定与 r2rpy 一致：固定坐标系 XYZ（roll,pitch,yaw），等价于动坐标系 ZYX（yaw,pitch,roll）。
	 * \note 对应公式（[w,x,y,z]）：
	 *       roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
	 *       pitch = asin (2*(w*y - z*x))
	 *       yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
	 * \param quat	输入四元数 [w, x, y, z]。
	 * \param rpy	输出roll,pitch,yaw（弧度）。
	 */
	void quat2rpy(float_def quat[4], float_def rpy[3]);

	/**
	 * \brief 复制四元数（w,x,y,z）。
	 */
	void quat_copy(const float_def src[4], float_def dst[4]);

	/**
	 * \brief 计算四元数模长（w,x,y,z）。
	 */
	float_def quat_norm(const float_def q[4]);

	/**
	 * \brief 四元数归一化（w,x,y,z）。
	 * \return 0 成功；非0 失败。
	 */
	int quat_normalize(const float_def q[4], float_def out[4]);

	/**
	 * \brief 计算四元数点积（w,x,y,z）。
	 */
	float_def quat_dot(const float_def q0[4], const float_def q1[4]);

	/**
	 * \brief 四元数数乘（w,x,y,z）。
	 */
	void quat_scale(const float_def q[4], float_def scale, float_def out[4]);

	/**
	 * \brief 四元数共轭（w,x,y,z）。
	 */
	void quat_conjugate(const float_def q[4], float_def out[4]);

	/**
	 * \brief 四元数乘法 out=q0*q1（w,x,y,z）。
	 */
	void quat_multiply(const float_def q0[4], const float_def q1[4], float_def out[4]);

	/**
	 * \brief 四元数球面线性插值（SLERP，w,x,y,z）。
	 * \return 0 成功；非0 失败。
	 */
	int quat_slerp(const float_def q0[4], const float_def q1[4], float_def s, float_def out[4]);

	/**
	 * \brief 数值限幅到闭区间 [low, high]。
	 */
	float_def clamp_value(float_def value, float_def low, float_def high);

	/**
	 * \brief 角度归一化到 [-PI, PI]。
	 */
	float_def wrap_to_pi(float_def angle);

	/**
	 * \brief		rpy角转化为齐次变换矩阵。
	 *
	 * \param rpy	绕固定坐标系的roll,pitch,yaw角度，单位：弧度。
	 * \param T		齐次变换矩阵。
	 */
	void rpy2tr(float_def rpy[3], float_def T[4][4]);

	/**
	 * \brief		rpy位姿描述角转化为齐次变换矩阵。
	 *
	 * \param xyzrpy	坐标x,y,z和绕固定坐标系的roll,pitch,yaw角度，单位：弧度。
	 * \param T		齐次变换矩阵。
	 */
	void xyzrpy2tr(float_def xyzrpy[6], float_def T[4][4]);

	/**
	 * \brief		齐次矩阵计算绕固定坐标系的位姿和rpy角。
	 *
	 * \param T		齐次矩阵。
	 * \param rpy	rpy角,单位：弧度。
	 */
	void tr2xyzrpy(float_def T[4][4], float_def xyzrpy[6]);

	/**
	 * \brief 绕x轴旋转的矩阵.
	 * \param rx	绕x轴旋转的角度，单位deg.
	 * \param R		绕x轴旋转的旋转矩阵.
	 */
	void rotx(float_def rx, float_def R[3][3]);

	/**
	 * \brief 绕y轴旋转的矩阵.
	 * \param ry	绕y轴旋转的角度，单位deg.
	 * \param R		绕y轴旋转的旋转矩阵.
	 */
	void roty(float_def ry, float_def R[3][3]);

	/**
	 * \brief 绕z轴旋转的矩阵.
	 * \param rz	绕z轴旋转的角度，单位deg.
	 * \param R		绕z轴旋转的旋转矩阵.
	 */
	void rotz(float_def rz, float_def R[3][3]);

	/**
	 * \brief 把XYZ欧拉角转换为旋转矩阵，XYZ欧拉角定义为：分别绕动坐标系的X,Y,Z轴旋转角度rx,ry,rz,对应的旋转矩阵为.
	 * Rxyz =Rotx*Roty*Rotz=
	 * [                           cos(ry)*cos(rz),                          -cos(ry)*sin(rz),          sin(ry)]
	 * [ cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry), cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz), -cos(ry)*sin(rx)]
	 * [ sin(rx)*sin(rz) - cos(rx)*cos(rz)*sin(ry), cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz),  cos(rx)*cos(ry)]
	 * \param Rx	绕动坐标系X轴旋转的角度，单位度
	 * \param Ry	绕动坐标系Y轴旋转的角度，单位度
	 * \param Rz	绕动坐标系Z轴旋转的角度，单位度
	 * \param R		输出的旋转矩阵
	 * \return		返回0执行成功，非零执行失败。
	 */
	int xyzEulerAngle2Rotation(float_def Rx, float_def Ry, float_def Rz, float_def R[3][3]);

	/**
	 * \brief 给定旋转矩阵，求解XYZ欧拉角，XYZ欧拉角定义为：分别绕动坐标系的X,Y,Z轴旋转角度rx,ry,rz.
	 *
	 * \param R		旋转矩阵。
	 * \param Rx	绕动坐标系X轴旋转的角度，单位度。求解范围Rx,Rz（-pi,pi],Ry[-pi/2,pi/2]
	 * \param Ry	绕动坐标系Y轴旋转的角度，单位度。
	 * \param Rz	绕动坐标系Z轴旋转的角度，单位度。
	 * \return		返回0执行成功，非零执行失败。
	 */
	int rotation2xyzEulerAngle(float_def R[3][3], float_def* Rx, float_def* Ry, float_def* Rz);

	/**
	 * \brief 把XYZ欧拉角转换为四元数。
	 *
	 * \note 角度定义与 xyzEulerAngle2Rotation 一致：绕动坐标系X/Y/Z轴旋转，单位度。
	 * \note 输出四元数分量顺序为 [w, x, y, z]。
	 * \param Rx	绕动坐标系X轴旋转的角度，单位度。
	 * \param Ry	绕动坐标系Y轴旋转的角度，单位度。
	 * \param Rz	绕动坐标系Z轴旋转的角度，单位度。
	 * \param quat	输出四元数 [w, x, y, z]。
	 * \return	返回0执行成功，非零执行失败。
	 */
	int xyzEulerAngle2quat(float_def Rx, float_def Ry, float_def Rz, float_def quat[4]);

	/**
	 * \brief 把四元数转换为XYZ欧拉角。
	 *
	 * \note 角度定义与 rotation2xyzEulerAngle 一致：绕动坐标系X/Y/Z轴旋转，单位度。
	 * \note 输入四元数分量顺序为 [w, x, y, z]。
	 * \param quat	输入四元数 [w, x, y, z]。
	 * \param Rx	绕动坐标系X轴旋转的角度，单位度。
	 * \param Ry	绕动坐标系Y轴旋转的角度，单位度。
	 * \param Rz	绕动坐标系Z轴旋转的角度，单位度。
	 * \return	返回0执行成功，非零执行失败。
	 */
	int quat2xyzEulerAngle(float_def quat[4], float_def* Rx, float_def* Ry, float_def* Rz);
	    
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
	int xyzEulerAngle2tr(float_def xe[6], float_def T[4][4]);

	/**
	 * \brief 把刚体运动旋转矩阵描述转换为x,y,z,rx,ry,rz笛卡尔坐标描述，刚体运动为transl(x)*transl(y)*transl(z)*rotx(rx)*roty(ry)*rotz(rz).
	 * \param T		齐次矩阵。
	 * \param xe	迪卡尔坐标，单位度。
	 * \return		返回0执行成功，非零执行失败。
	 */
	int tr2xyzEulerAngle(float_def T[4][4], float_def xe[6]);



	/**
	 * \brief 向量单位化。
	 *
	 * \param vec1	输入向量。
	 * \param vec2	单位化后的输出向量。
	 * \return 返回值0执行成功，返回值非0执行失败。
	 * \retval 1 输入向量为0。
	 */
	int vec3_unit(float_def* vec1, float_def* vec2);

	/**
	*@brief			3x3反对称矩阵对应的三维向量.
	*@param[in]		so3Mat		反对称矩阵.
	*@param[out]	omg			3维向量.
	*@return		无返回值.
	*@note:
	*@warning:
	*/
	void so3_vec(float_def so3Mat[3][3], float_def omg[3]);


	/**
	 * \brief  由给定ω,θ生成为反对称矩阵指数 exp(ωθ)
	 *
	 * \param vec	3维向量
	 * \param theta	关节量。
	 * \param R		姿态矩阵。
	 */
	void vec_exp3(float_def vec[3], float_def theta, float_def exp3[3][3]);


	/**
	*@brief			计算旋转矩阵的对数，即由旋转矩阵求对应的反对称矩阵.
	*@param[in]		R		the rotation matrix.
	*@param[out]	so3Mat	matrix logarithm.
	*@return		No return value.
	*@note:
	*@warning:
	*/
	void matrix_log3(float_def R[3][3], float_def so3Mat[3][3]);


	/**
	 * \brief	由向量和位移计算刚体变换矩阵。
	 *
	 * \param type	类型：0--旋转轴，1--平移轴。
	 * \param vec	轴的方向矢量。
	 * \param q		轴上一点的坐标。
	 * \param theta	关节量。
	 * \param exp6	刚体的齐次变换矩阵。
	 */
	void vec_exp6(int type, float_def vec[3], float_def q[3], float_def theta, float_def exp6[4][4]);

	/**
	*@brief 计算计算齐次变换对应的 6 x 6 伴随变换 [AdT ]
	*@param[in]		T		齐次变换矩阵.
	*@param[out]	AdT		6x6伴随映射矩阵 [AdT ].
	*@return		无返回值.
	*@note:
	*@warning:
	*/
	void Adjoint(float_def T[4][4], float_def AdT[6][6]);

	void jacobian_body(int JointNum, float_def Blist[6][6], float_def thetalist[6], float_def  Jb[6][6]);

	/**
	 * \brief 计算空间雅可比矩阵.
	 *
	 * \param JointNum 关节数量
	 * \param qs		轴上一点的坐标
	 * \param ws		轴线单位方向矢量
	 * \param thetalist 关节角度
	 * \param Js        雅可比矩阵(注意，结果根据关节数量确定，存储在前JointNum列)
	 */
	void jacobian_space(int JointNum, float_def Slist[6][6], float_def* thetalist, float_def Js[6][6]);

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
	void RotToAxisAng(float_def R[3][3], float_def omghat[3], float_def* theta);



	/**
	 * \brief 求解paden_kahan子问题1，参考《机器人操作的数学导论实现》.
	 *
	 * \param w	旋转轴的单位方向向量
	 * \param u	点（或向量）旋转前的坐标
	 * \param v	点（或向量）旋转后的坐标
	 * \return 待求解的旋转角，单位:弧度
	 */
	float_def paden_kahan1(float_def w[3], float_def u[3], float_def v[3]);

	/**
	 * \brief 求解paden_kahan子问题2，参考《机器人操作的数学导论》实现.
	 *
	 * \param w2 第一个旋转轴的方向矢量。
	 * \param w2 第二个旋转轴的方向矢量。
	 * \param u  旋转
	 * \param u	点（或向量）旋转前的坐标.
	 * \param v	点（或向量）旋转后的坐标.
	 * \return 返回0执行成功，返回非0执行失败。
	 * \retval 1 参数错误，两个轴的方向向量平行。
	 * \retval 2 无解，所给参数不存在解。
	 * \atention 两个轴的方向矢量不能平行，平行时会返回错误参数1。
	 */
	int paden_kahan2(float_def w2[3], float_def w1[3], float_def u[3], float_def v[3], float_def theta[2][2]);

	/**
	 * \brief		齐次变换矩阵乘法（计算速度较快），该函数利用其特殊性，加快运算速度。
	 *
	 * \param a		齐次变换矩阵a.
	 * \param b		齐次变换矩阵b.
	 * \param c		齐次变换矩阵c.
	 */
	void htm_matrix_mult(float_def a[][4], float_def b[][4], float_def c[][4]);


	/**
	 * \brief 高斯消去法解线性方程组（注意，没有选列主元）.
	 *
	 * \return 1	出现对角线元素为0,方程无法求解
	 */
	int gauss_solve_linear_equation(float_def* A, int NUM, float_def* b, float_def* X);

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
    int matrix_pinv(const float_def* a, int m, int n, float_def tol, float_def* b);

	/**
    * \brief 六维力传感器后方工具质心校准
    * \param 输入forceIni:6组静态试验下的六维力传感器读数，每一行为一组数据，，国际单位制
    * \param 输出centroid：前3个元素为传感器坐标系的工具质心位置。
    */
	int toolCentroidCalibrate(double forceIni[6][6], double centroid[6]);

	/**
    * \brief 六维力传感器零点和后方工具重力辨识
    * \param 输入1：forceIni:n组(n=6-14)静态试验下的六维力传感器读数，每一行为一组数据，，国际单位制
    * \param 输入2：rpy：静态试验时，工具坐标系相对机器人基坐标系的姿态角(固定参考系，次序为XYZ轴),n*3
    * \param 输入3：rpySensor：传感器坐标系下，工具坐标系的姿态角rpy
    * \param 输出1:return工具重力大小
    * \param 输出2:SensorZero传感器零点(本次试验下的零点数值)
    */
	double toolGravityAndSensorZero(double forceIni[6][6], double rpy[6][3], double rpySensor[3], double centroid[6], double SensorZero[6]);

	/**
	 * \brief 六维力传感器零点通电更新
	 * \param 输入1：force:开机通电后机械臂静态试验下的六维力传感器读数（最好是滤波后数据）
	 * \param 输入2：rpy：静态试验时，工具坐标系相对机器人基坐标系的姿态角rpy(固定参考系，次序为XYZ轴)
	 * \param 输入3：rpySensor：传感器坐标系下，工具坐标系的姿态角rpy
	 * \param 输入4:工具重力大小，质心位置（传感器坐标系下数值）
	 * \param 输出1:SensorZero传感器零点(本次通电下更新的零点数值)
	*/
	int sensorZeroUpdata(double force[6], double rpy[3], double rpySensor[3], double gravityAll[4], double sensorZero[6]);

	/**
	* \brief 计算工具重力负载（传感器坐标系下）
	* \param 输入1：rpy: 基坐标系下，工具姿态rpy，由正向运动学给出
	* \param 输入2：rpySensor： 传感器坐标系下，工具姿态rpy,固定值
	* \param 输入3：gravityAll  工具重力大小，质心位置（传感器坐标系下），提前辨识给出
	* \param 输出1:G_sensor     传感器坐标系中的工具重力负载
	*/
	int toolGravitySensor(double rpy[3], double rpySensor[3], double gravityAll[4], double G_sensor[6]);

	/**
	 * \brief 六维力传感器环境有效负载的提取
	 *
	 * \param force    传感器读数
	 * \param sensorZero     传感器零点
	 * \param rpy            基坐标系下，工具姿态rpy
	 * \param rpySensor      传感器坐标系下，工具姿态rpy
	 * \param gravityAll     工具重力大小，质心位置（传感器坐标系下）
	 * \param load           环境接触力
	 */
	int contactForceGet(double force[6], double sensorZero[6], double rpy[3], double rpySensor[3], double gravityAll[4], double load[6]);

#ifdef __cplusplus
}
#endif

#endif // !MAHTLIB_H_

