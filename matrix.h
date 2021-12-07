#ifndef __MATRIX_H
#define __MATRIX_H

#include "stdio.h"
#include "stdlib.h"
#include "sys.h"
#include "string.h"
#include "math.h"
#include "malloc.h"

// �������ݽṹ
typedef struct
{
	u32 nrow;
	u32 ncolumn;
	float *data;
}matrix;

// ����Ļ�����������
// ���þ�������ݣ��ɹ�����0��ʧ�ܷ���-1
int matrix_set_data(matrix *m, float *val, u32 list_size);

// �����о��������ȫ����0���ɹ�����0��ʧ�ܷ���-1
int matrix_set_zero(matrix *m);

// ������λ�󣬳ɹ��򷵻ط�����ڴ��matrix�ṹ��ָ�룬ʧ���򷵻ؿ�ָ��
matrix* matrix_create_identity(u32 row_column);

// �����ʼ�����ɹ��򷵻�0��ʧ���򷵻�-1
matrix* matrix_init(u32 num_row, u32 num_column);

// �ͷž����ڴ棬�ɹ��򷵻�0��ʧ���򷵻�-1
int matrix_free(matrix **m);

// �ھ���ĳ��λ��д��ֵ���ɹ��򷵻�0��ʧ���򷵻�-1
int matrix_write(matrix *m, u32 row, u32 column, float val);

// ��ȡ����ĳ��λ�õ����ݲ�����
float matrix_read(matrix *m, u32 row, u32 column);

// ������������е�����Ϊ���������ԽǾ���
matrix* create_diagonal(float* list, u32 number);

// ����Ļ������㷽��

// ���󻥼ӣ��ɹ��򷵻�0�����򷵻�-1
int matrix_add(matrix *a, matrix *b, matrix *result);

// ���󻥼����ɹ��򷵻�0�����򷵻�-1
int matrix_substract(matrix *a, matrix *b, matrix *result);

// ���󻥳ˣ��ɹ��򷵻�0�����򷵻�-1
int matrix_multiply(matrix *a, matrix *b, matrix *result);

// ����˱������ɹ��򷵻�0�����򷵻�-1
int matrix_multiply_scalar(matrix *m, float val);

// ������һ������������������Ҳ������������
int vector_normalize(matrix *m);

// ���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4��
float invSqrt(float x);

// ����ת�ã�
int matrix_transpose(matrix *m, matrix *result);

// �������棬�˴����õ�����������İ취���ɹ��򷵻�0�����򷵻�-1
int matrix_inverse(matrix *m, matrix* inversed);

// �������棬�˴����õ���Gauss-Jordan����Ԫ��ȥ�����ɹ��򷵻�0�����򷵻�-1
int matrix_inverse_gauss(matrix *m, matrix* inversed);

// ���������ʽֵ���˴����õ��ǵݹ�ķ�����������ʽֵ
float matrix_determinant(matrix *m);

// ���������ʽֵ���˴����õ����б任������Ԫ��ȥ�����ķ�������ԭ�����Ϊ�����Ǿ�����������ʽֵ
float matrix_determinant_gauss(matrix *m);

// ���������ʽֵ(��������Ϊ���룬�еݹ�)
float list_determinant(float *list, u32 number);

// ����ļ�
float matrix_trace(matrix *m);

// ������ӷ���(ȥ����ĳ���Լ�ĳ�е�Ԫ��)
int get_sub_square_matrix(float *list, float *result, u32 number, int exclude_row, int exclude_column);

// ��������
int matrix_adjoint(matrix *m, matrix *m_star);

// �޸Ĵ��������ǴӴ������ȡС����
// ���chooseΪ1, ����ԭ�����ĳ��λ������С��������ݽ�ȥ���Եõ��µĴ���󣬳ɹ��򷵻�0��ʧ���򷵻�-1
// ���chooseΪ0, ����ԭ�����ĳ��λ��ȡ��������䵽С������Եõ��µ�С���󣬳ɹ��򷵻�0��ʧ���򷵻�-1
int matrix_padding(matrix *big, matrix *patch, u32 start_row, u32 start_column, u32 choose_big);

// 3��������ˣ�a*(���)b=c
int matrix_cross_product_3(matrix *a, matrix *b, matrix *c);

// ��ӡ���������Ա㷢��NaN
void printf_matrix(matrix *m);
#endif // __MATRIX_H

