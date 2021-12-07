#ifndef __MATRIX_H
#define __MATRIX_H

#include "stdio.h"
#include "stdlib.h"
#include "sys.h"
#include "string.h"
#include "math.h"
#include "malloc.h"

// 矩阵数据结构
typedef struct
{
	u32 nrow;
	u32 ncolumn;
	float *data;
}matrix;

// 矩阵的基本操作方法
// 设置矩阵的内容，成功返回0，失败返回-1
int matrix_set_data(matrix *m, float *val, u32 list_size);

// 将已有矩阵的内容全部置0，成功返回0，失败返回-1
int matrix_set_zero(matrix *m);

// 创建单位阵，成功则返回分配好内存的matrix结构体指针，失败则返回空指针
matrix* matrix_create_identity(u32 row_column);

// 矩阵初始化，成功则返回0，失败则返回-1
matrix* matrix_init(u32 num_row, u32 num_column);

// 释放矩阵内存，成功则返回0，失败则返回-1
int matrix_free(matrix **m);

// 在矩阵某个位置写入值，成功则返回0，失败则返回-1
int matrix_write(matrix *m, u32 row, u32 column, float val);

// 读取矩阵某个位置的数据并返回
float matrix_read(matrix *m, u32 row, u32 column);

// 以输入的数组中的数据为基础构建对角矩阵
matrix* create_diagonal(float* list, u32 number);

// 矩阵的基本运算方法

// 矩阵互加，成功则返回0，否则返回-1
int matrix_add(matrix *a, matrix *b, matrix *result);

// 矩阵互减，成功则返回0，否则返回-1
int matrix_substract(matrix *a, matrix *b, matrix *result);

// 矩阵互乘，成功则返回0，否则返回-1
int matrix_multiply(matrix *a, matrix *b, matrix *result);

// 矩阵乘标量，成功则返回0，否则返回-1
int matrix_multiply_scalar(matrix *m, float val);

// 向量归一化，可以是行向量，也可以是列向量
int vector_normalize(matrix *m);

// 快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍
float invSqrt(float x);

// 矩阵转置，
int matrix_transpose(matrix *m, matrix *result);

// 矩阵求逆，此处采用的是求伴随矩阵的办法，成功则返回0，否则返回-1
int matrix_inverse(matrix *m, matrix* inversed);

// 矩阵求逆，此处采用的是Gauss-Jordan列主元消去法，成功则返回0，否则返回-1
int matrix_inverse_gauss(matrix *m, matrix* inversed);

// 求方阵的行列式值，此处采用的是递归的方法计算行列式值
float matrix_determinant(matrix *m);

// 求方阵的行列式值，此处采用的是行变换（列主元消去法）的方法，将原矩阵变为上三角矩阵，再求行列式值
float matrix_determinant_gauss(matrix *m);

// 求方阵的行列式值(以数组作为输入，有递归)
float list_determinant(float *list, u32 number);

// 求方阵的迹
float matrix_trace(matrix *m);

// 求方阵的子方阵(去除了某行以及某列的元素)
int get_sub_square_matrix(float *list, float *result, u32 number, int exclude_row, int exclude_column);

// 求伴随矩阵
int matrix_adjoint(matrix *m, matrix *m_star);

// 修改大矩阵或者是从大矩阵中取小矩阵，
// 如果choose为1, 则在原矩阵的某个位置填入小矩阵的数据进去，以得到新的大矩阵，成功则返回0，失败则返回-1
// 如果choose为0, 则在原矩阵的某个位置取出数据填充到小矩阵里，以得到新的小矩阵，成功则返回0，失败则返回-1
int matrix_padding(matrix *big, matrix *patch, u32 start_row, u32 start_column, u32 choose_big);

// 3阶向量叉乘，a*(叉乘)b=c
int matrix_cross_product_3(matrix *a, matrix *b, matrix *c);

// 打印整个矩阵，以便发现NaN
void printf_matrix(matrix *m);
#endif // __MATRIX_H

