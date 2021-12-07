#include "matrix.h"
// ����Ļ�����������
// �����ǰ��������ȵ�ԭ�����洢�ģ��⼴һ��һ�еĴ洢��
// ���þ�������ݣ��ɹ�����0��ʧ�ܷ���-1��Ĭ���������m���Ѿ���ʼ�����˵ľ���
int matrix_set_data(matrix *m, float *val, u32 list_size)
{
	int i = 0;
	if(m && m->data && val)
	{
		// sizeofֻ���������飬������ʾ����С����ֻ������ָ�룬��ֻ����ʾ���ָ���Ƕ����ֽ�
		if(list_size == (m->ncolumn*m->nrow))
		{
			for(i=0;i<list_size;i++)
			{
				m->data[i] = val[i];
			}
			return 0;
		}
		else
	  {
		  printf("the size of val is not matched with size of matrix!");
	  }
	}
	else
	{
		printf("null matrix pointer or the data pointer of matrix is null pointer!");
	}
	return -1;
}

// ������һ������������������Ҳ���������������ɹ�����0��ʧ�ܷ���-1
int vector_normalize(matrix *m)
{
	int i = 0;
	float temp = 0.0f;
	if(m && m->data)
	{
		if(m->ncolumn == 1 || m->nrow == 1)
		{
			for(i=0;i<(m->ncolumn)*(m->nrow);i++)
		  {
			  temp += (m->data[i])*(m->data[i]);
		  }
		  if(temp)
		  {
		  	matrix_multiply_scalar(m, invSqrt(temp));
				return 0;
	  	}
	  	else
	  	{
	  		printf("the sum of square is zero, can not normalized!");
	  	}
		}
		else
		{
			printf("the matrix inputed is not a vector!");
		}
	}
	else
	{
		printf("the matrix is null pointer!");
	}
	return -1;
}

// ���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4��
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	//��floatתΪlong��i�����������ģ���ԭΪfloat��x�������ֽ���long�Ͷ�ȡ
	long i = *(long*)&y;
	//0x5f3759df��ȥi��һ��
	i = 0x5f3759df - (i>>1);
	//��longתΪfloat���µ�y�����������ģ���ԭΪlong��i�������ֽ���float�Ͷ�ȡ
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// ������λ�󣬳ɹ��򷵻ط�����ڴ��matrix�ṹ��ָ�룬ʧ���򷵻ؿ�ָ��
matrix* matrix_create_identity(u32 row_column)
{
	int i = 0;
	matrix *result = NULL;
	result = matrix_init(row_column, row_column);
	if(result)
	{
		for(i=0;i<row_column;i++)
		{
			result->data[i*row_column+i] = 1;
		}
	}
	else
	{
		printf("null matrix pointer, memory malloc failed!");
	}
	return result;
}

// �����о��������ȫ����0���ɹ�����0��ʧ�ܷ���-1��
int matrix_set_zero(matrix *m)
{
	if(m)
	{
		if(m->data)
		{
			mymemset(m->data, 0, (m->ncolumn * m->nrow)*sizeof(float));
			return 0;
		}
		else
		{
			printf("null matrix data pointer!");
		}
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// ����˱������ɹ��򷵻�0�����򷵻�-1
int matrix_multiply_scalar(matrix *m, float val)
{
	int i = 0;
	if(m)
	{
		if(m->data)
		{
			for(i=0;i<(m->nrow*m->ncolumn);i++)
			{
				m->data[i] *= val;
			}
			return 0;
		}
		else
		{
			printf("null matrix data pointer!");
		}
	}
	else
	{
		printf("null matrix pointer, memory malloc failed!");
	}
	return -1;
}

// �����ʼ�������ɹ����򷵻�һ��matrix�ṹ��ָ�룬ʧ���򷵻ؿ�ָ��
matrix* matrix_init(u32 num_row, u32 num_column)
{
	matrix *result = NULL;
	result = (matrix*)mymalloc(SRAMIN, sizeof(matrix));
	// ����������л�������ĿΪ0�����
	if(result && num_column && num_row)
	{
		result->nrow = num_row;
		result->ncolumn = num_column;
		result->data = (float*)mymalloc(SRAMIN, (num_row * num_column)*sizeof(float));
	  if(result->data)
		{
			mymemset(result->data, 0, (result->ncolumn * result->nrow)*sizeof(float));
		}
		else
		{
			printf("null matrix data pointer, memory malloc failed!");
		}
	}
	else
	{
		printf("null matrix pointer, memory malloc failed!");
	}
	return result;
}

// �ͷž����ڴ�
int matrix_free(matrix **m)
{
	if(*m)
	{
		if((*m)->data)
		{
			myfree(SRAMIN, (*m)->data);
		  (*m)->data = NULL;
		}
		myfree(SRAMIN, *m);
		*m = NULL;
		return 0;
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// �ھ���ĳ��λ��д��ֵ���ɹ��򷵻�0��ʧ���򷵻�-1
int matrix_write(matrix *m, u32 row, u32 column, float val)
{
	if(m)
	{
		if(m->data)
		{
			m->data[row * m->ncolumn + column] = val;
			return 0;
		}
	  else
		{
			printf("null matrix data pointer!");
		}
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// ��ȡ����ĳ��λ�õ����ݲ����أ��˴������ǳ��ִ��󣬷��غ�ֵ��
// ����ֱ�ӽ�������
float matrix_read(matrix *m, u32 row, u32 column)
{
	if(m)
	{
		if(m->data)
		{
			return m->data[row * m->ncolumn + column];
		}
	  else
		{
			printf("null matrix data pointer!");
		}
	}
	else
	{
		printf("null matrix pointer!");
	}
	exit(1);
}

// ������������е�����Ϊ���������ԽǾ���
matrix* create_diagonal(float* list, u32 number)
{
	int i = 0;
	matrix* result = matrix_init(number, number);
	if(result && result->data)
	{
		for(i=0;i<number;i++)
		{
			result->data[i*number+i] = list[i];
		}
	}
	else
	{
		printf("memory malloc failed!");
	}
	return result;
}


// ����Ļ������㷽��

// ����ӣ��ɹ��򷵻�0�����򷵻�-1
int matrix_add(matrix *a, matrix *b, matrix *result)
{
	int i = 0;
	int j = 0;
	if(a && b && result)
	{
		if((a->ncolumn == b->ncolumn) && (a->ncolumn == result->ncolumn)
			&& (a->nrow == b->nrow) && (a->nrow == result->nrow) 
		&& a->data && b->data && result->data)
		{
			for(i=0;i<a->nrow;i++)
			{
				for(j=0;j<a->ncolumn;j++)
				{
					result->data[i*a->ncolumn+j] = a->data[i*a->ncolumn+j] + b->data[i*a->ncolumn+j];
				}
			}
			return 0;
		}
	  else
		{
			printf("the parameter of row or column is not matched!");
			printf("a->ncolumn: %d", a->ncolumn);
			printf("a->nrow: %d", a->nrow);
			printf("b->ncolumn: %d", b->ncolumn);
			printf("b->nrow: %d", b->nrow);
			printf("result->ncolumn: %d", result->ncolumn);
			printf("result->nrow: %d", result->nrow);
			exit(1);
		}
	}
	else
	{
		printf("null matrix pointer!");
		exit(1);
	}
	return -1;
}

// ��������ɹ��򷵻�0�����򷵻�-1
int matrix_substract(matrix *a, matrix *b, matrix *result)
{
	int i = 0;
	int j = 0;
	if(a && b && result)
	{
		if((a->ncolumn == b->ncolumn) && (a->ncolumn == result->ncolumn)
			&& (a->nrow == b->nrow) && (a->nrow == result->nrow) 
		&& a->data && b->data && result->data)
		{
			for(i=0;i<a->nrow;i++)
			{
				for(j=0;j<a->ncolumn;j++)
				{
					result->data[i*a->ncolumn+j] = a->data[i*a->ncolumn+j] - b->data[i*a->ncolumn+j];
				}
			}
			return 0;
		}
		else
		{
			printf("the parameter of row or column is not matched!");
			exit(1);
		}
	}
	else
	{
		printf("null matrix pointer!");
		exit(1);
	}
	return -1;
}

// ����ˣ��ɹ��򷵻�0�����򷵻�-1
int matrix_multiply(matrix *a, matrix *b, matrix *result)
{
	int i = 0;
	int j = 0;
	int k = 0;
	float dot = 0.0;
	if(a && b && result)
	{
		if((a->ncolumn == b->nrow) && (a->nrow == result->nrow)
			&& (b->ncolumn == result->ncolumn) 
		&& a->data && b->data && result->data)
		{
			for(i=0;i<a->nrow;i++)
			{
				for(j=0;j<b->ncolumn;j++)
				{
					dot = 0.0f;
					for(k=0;k<a->ncolumn;k++)
					{
						dot += a->data[i*a->ncolumn+k] * b->data[k*b->ncolumn+j];
					}
					result->data[i*b->ncolumn+j] = dot;
				}
			}
			
			return 0;
		}
		else
		{
			printf("the parameter of row or column is not matched!");
			exit(1);
		}
	}
	else
	{
		printf("null matrix pointer!");
		exit(1);
	}
	return -1;
}

// ����ת�ã�m���Ѿ���ʼ�����˵ľ������ɹ��򷵻�matrix�ṹ��ָ�룬���򷵻ؿ�ָ��
int matrix_transpose(matrix *m, matrix *result)
{
	int i = 0;
	int j = 0;
	if(m && result && (m->nrow == result->ncolumn) && (m->ncolumn == result->nrow))
	{
		for(i=0;i<m->nrow;i++)
		{
			for(j=0;j<m->ncolumn;j++)
			{
				result->data[j*result->ncolumn+i] = m->data[i*m->ncolumn+j];
			}
		}
		return 0;
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// ��������
int matrix_inverse(matrix *m, matrix *inversed)
{
	float temp_det = 0.0f;
	if(m && inversed)
	{
		temp_det = matrix_determinant(m);
		// ������ʽ��ֵ��Ϊ0��ʱ�򣬲ſ��Լ��������
		if(temp_det)
		{
			matrix_adjoint(m, inversed);
			// ����������������ʽֵ�����õ������
			matrix_multiply_scalar(inversed, 1.0f/temp_det);
			return 0;
		}
		else
		{
			printf("determinant is zero, it is a singular matrix!");
		}
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// ���������ʽֵ
float matrix_determinant(matrix *m)
{
	float result = 0.0f;
	if(m && (m->ncolumn == m->nrow) && (m->ncolumn))
	{
		if(m->data)
		{
			if(m->ncolumn == 1)
			{
				result = m->data[0];
			}
			else
			{
				result = list_determinant(m->data, m->nrow);
			}
		}
		else
		{
			printf("null matrix data pointer!");
		}
	}
	else
	{
		printf("null matrix pointer or not square matrix!");
	}
	return result;
}

// ���������ʽֵ(��������Ϊ���룬�еݹ�)
float list_determinant(float *list, u32 number)
{
	float result = 0.0f;
	int i = 0;
	float *temp = mymalloc(SRAMIN, sizeof(float)*(number-1)*(number-1));
	
	// �������С����1*1�ģ�����еݹ�
	if(temp && list)
	{
		if(number == 1)
	  {
		  result += list[0];
	  }
		else
		{
			// �ӵ�һ�п�ʼչ�����������������ʽ��ֵ
			for(i=0;i<number;i++)
			{
				// ��Ϊ0������������ʽ�Ͳ��ؼ�����
				if(list[i])
				{
					get_sub_square_matrix(list, temp, number, 0, i);
					// �õ��Ӿ���֮�󣬽��еݹ�
					if(i%2)
					{
						result -= list[i]*list_determinant(temp, number-1);
					}
					else
					{
						result += list[i]*list_determinant(temp, number-1);
					}	
				}
			}
		}
    // �ǵ��ͷ��ڴ�
	  myfree(SRAMIN, temp);
	  temp = NULL;		
	}
	else
	{
		printf("memory malloc failed!");
		exit(1);
	}
	return result;
}

// ������ӷ���(ȥ����ĳ���Լ�ĳ�е�Ԫ��)
int get_sub_square_matrix(float *list, float *temp, u32 number, int exclude_row, int exclude_column)
{
	int t = 0;
	int i = 0;
	int j = 0;
	if(temp && list)
	{
		for(i=0;i<number;i++)
		{
			if(i==exclude_row)
			{
				continue;
			}
			for(j=0;j<number;j++)
			{
				if(j!=exclude_column)
			  {
			  	temp[t++] = list[i*number+j];
			  }
			}
		}
		return 0;
	}
	else
	{
		printf("memory malloc failed!");
	}
	return -1;
}

// ��������
int matrix_adjoint(matrix *m, matrix *m_star)
{
	float temp_det = 0.0f;
	float* temp = NULL;
	int i = 0;
	int j = 0;
	// �����������Ԫ�ظ���Ϊ1����ֱ�ӷ��ذ������
	// �����Ƿ��󣬲�����������
	if(m && (m->ncolumn==m->nrow) && temp && (m->ncolumn==m_star->nrow) && (m->nrow==m_star->ncolumn))
	{
		if(m->ncolumn == 1)
		{
			m_star->data[0] = 1;
			return 0;
		}
		temp = mymalloc(SRAMIN, (m->nrow-1)*(m->nrow-1)*sizeof(float));
		mymemset(temp, 0, (m->nrow-1)*(m->nrow-1)*sizeof(float));
		for(i=0;i<m->nrow;i++)
	  {
	  	for(j=0;j<m->ncolumn;j++)
			{
				get_sub_square_matrix(m->data, temp, m->nrow, i, j);
				// ��������ʽ
				temp_det = list_determinant(temp, m->nrow-1);
				// �����������ʽ��˳��ѽ���������ת��
				if((i+j)%2)
				{
					m_star->data[j*(m_star->ncolumn)+i] = (-1)*temp_det;
				}
				else
				{
					m_star->data[j*(m_star->ncolumn)+i] = temp_det;
				}
			}
	  }
		myfree(SRAMIN, temp);
		temp = NULL;
		return 0;
	}
	else
	{
		printf("null matrix pointer or not square matrix!");
	}
	return -1;
}

// �޸Ĵ��������ǴӴ������ȡС����
// ���chooseΪ1, ����ԭ�����ĳ��λ������С��������ݽ�ȥ���Եõ��µĴ���󣬳ɹ��򷵻�0��ʧ���򷵻�-1
// ���chooseΪ0, ����ԭ�����ĳ��λ��ȡ��������䵽С������Եõ��µ�С���󣬳ɹ��򷵻�0��ʧ���򷵻�-1
int matrix_padding(matrix *big, matrix *patch, u32 start_row, u32 start_column, u32 choose_big)
{
	int i = 0;
	int j = 0;
	if(big && patch && big->data && patch->data)
	{
		if(start_row<(big->nrow) && start_column<(big->ncolumn) 
		  && (patch->nrow)<=((big->nrow)-start_row) 
		  && (patch->ncolumn)<=((big->ncolumn)-start_column))
		{
			if(choose_big)
			{
				for(i = 0;i<patch->nrow;i++)
				{
					for(j = 0;j<patch->ncolumn;j++)
					{
						big->data[(start_row+i)*(big->ncolumn)+(start_column+j)] = patch->data[i*(patch->ncolumn)+j];
					}
				}
			}
			else
			{
				for(i = 0;i<patch->nrow;i++)
				{
					for(j = 0;j<patch->ncolumn;j++)
					{
						patch->data[i*(patch->ncolumn)+j] = big->data[(start_row+i)*(big->ncolumn)+(start_column+j)];
					}
				}
			}
			return 0;
		}
		else
		{
			printf("wrong row or column parameter!");
		}
	}
	else
	{
		printf("null matrix pointer or null data pointer!");
	}
	return -1;
}

// 3��������ˣ�a*(���)b=c
int matrix_cross_product_3(matrix *a, matrix *b, matrix *c)
{
	if(a && b && c)
	{
		c->data[0] = a->data[1]*b->data[2] - a->data[2]*b->data[1];
		c->data[1] = a->data[2]*b->data[0] - a->data[0]*b->data[2];
		c->data[2] = a->data[0]*b->data[1] - a->data[1]*b->data[0];
		return 0;
	}
	else
	{
		printf("null matrix pointer!");
	}
	return -1;
}

// ���������ʽֵ���˴����õ����б任������Ԫ��ȥ�����ķ�������ԭ�����Ϊ�����Ǿ�����������ʽֵ
float matrix_determinant_gauss(matrix *m)
{
	int i, j, n, max_row, swap_f = 0;
	float max, data_temp = 0.0f;
	float result = 1.0f;
	matrix *temp = NULL; 
	if(m && m->nrow == m->ncolumn)
	{
		temp = matrix_init(m->nrow, m->ncolumn);
		matrix_padding(temp, m, 0, 0, 1);
		for(i = 0;i<temp->nrow-1;i++)
		{
			// �ӵ�һ�п�ʼ������Ѱ����һ�о���ֵ�����Ǹ�ֵ�����Ҽ�¼�����ڵ��е��кţ�
			// Ȼ�����ڵĵ�i�к����¼���к����н���λ��
			max = fabsf(temp->data[i*(temp->ncolumn)+i]);
			max_row = i;
			// ѡ�����ֵ���ģ���Ϊ������㷨���ȶ��ԣ��ܶ�ʱ�򣬲�Ӱ�������
			// ���ѡ����Ԫ�ر�С����ô�����ٽ��г���ʱ�򣬾ͻ�Ѿ����е�Ԫ�ر���ر�������������
			// �������;����е�Ԫ�صĸı�ķ���
			for(j = i+1;j<temp->nrow;j++)
			{
				// �����i��ʾ�ڼ��У���Ϊǰ���i�ȱ�ʾ�У��ֱ�ʾ�У�ǰ��������ǶԽ����ϵ�Ԫ�أ�
				if(max<fabsf(temp->data[j*(temp->ncolumn)+i]))
				{
					max = fabsf(temp->data[j*(temp->ncolumn)+i]);
					max_row = j;
				}
			}
			if(!max)
			{
				printf("det of this matrix is zero!");
				return 0.0f;
			}
			// �ѵ�i�к����ֵ���ڵ����н���λ��
			if(i != max_row)
			{
				for(j = i;j<temp->ncolumn;j++)
				{
					data_temp = temp->data[i*temp->ncolumn+j];
					temp->data[i*temp->ncolumn+j] = temp->data[max_row*temp->ncolumn+j];
					temp->data[max_row*temp->ncolumn+j] = data_temp;
				}
				swap_f += 1;
			}
			// �ѵ�i�����µ������еĵ�i�У�����Ϊ0
			for(j = i+1;j<temp->nrow;j++)
			{
				// ���Ѿ�Ϊ0����û��Ҫ�ٽ��м�����
				if(temp->data[j*temp->ncolumn+i])
				{
					data_temp = temp->data[j*temp->ncolumn+i]/temp->data[i*temp->ncolumn+i]*(-1.0f);
				  for(n = i;n<temp->ncolumn;n++)
				  {
					  temp->data[j*temp->ncolumn+n] += temp->data[i*temp->ncolumn+n] * data_temp;
				  }
				}
			}
		}
		for(i=0;i<temp->nrow;i++)
		{
			result *= temp->data[i*temp->ncolumn+i];
		}
		matrix_free(&temp);
		if(swap_f%2)
		{
			result *= -1.0f;
		}
		return result;
	}
	else
	{
		printf("error:In det_mat (m->column != m->row)\n");
    exit(1);
	}
}

// �������棬�˴����õ���Gauss-Jordan����Ԫ��ȥ�����ɹ��򷵻�0�����򷵻�-1
int matrix_inverse_gauss(matrix *m, matrix* inversed)
{
	int i, j, n, max_row = 0;
	float max, data_temp = 0.0f;
	matrix *temp = NULL;
  if(!matrix_determinant_gauss(m))
	{
		printf("singular matrix! can not be inversed!");
		return -1;
	}		
	if(m && inversed && m->nrow == m->ncolumn && inversed->nrow == m->nrow)
	{
		// �����������������1���γɵ�λ��
		for(i = 0;i<inversed->nrow;i++)
		{
			inversed->data[i*inversed->ncolumn+i] = 1.0f;
		}
		temp = matrix_init(m->nrow, m->ncolumn+inversed->ncolumn);
		matrix_padding(temp, m, 0, 0, 1);
		matrix_padding(temp, inversed, 0, m->ncolumn, 1);
		// �ȰѾ���Ϊ�����Ǿ���
		for(i = 0;i<temp->nrow-1;i++)
		{
			// �ӵ�һ�п�ʼ������Ѱ����һ�о���ֵ�����Ǹ�ֵ�����Ҽ�¼�����ڵ��е��кţ�
			// Ȼ�����ڵĵ�i�к����¼���к����н���λ��
			max = fabsf(temp->data[i*(temp->ncolumn)+i]);
			max_row = i;
			for(j = i+1;j<temp->nrow;j++)
			{
				// �����i��ʾ�ڼ��У���Ϊǰ���i�ȱ�ʾ�У��ֱ�ʾ�У�ǰ��������ǶԽ����ϵ�Ԫ�أ�
				if(max<fabsf(temp->data[j*(temp->ncolumn)+i]))
				{
					max = fabsf(temp->data[j*(temp->ncolumn)+i]);
					max_row = j;
				}
			}
			// �ѵ�i�к����ֵ���ڵ����н���λ��
			if(i != max_row)
			{
				for(j = i;j<temp->ncolumn;j++)
				{
					data_temp = temp->data[i*temp->ncolumn+j];
					temp->data[i*temp->ncolumn+j] = temp->data[max_row*temp->ncolumn+j];
					temp->data[max_row*temp->ncolumn+j] = data_temp;
				}
			}
			// �ѵ�i�����µ������еĵ�i�У�����Ϊ0
			for(j = i+1;j<temp->nrow;j++)
			{
				// ���Ѿ�Ϊ0����û��Ҫ�ٽ��м�����
				if(temp->data[j*temp->ncolumn+i])
				{
					data_temp = temp->data[j*temp->ncolumn+i]/temp->data[i*temp->ncolumn+i]*(-1.0f);
				  for(n = i;n<temp->ncolumn;n++)
				  {
					  temp->data[j*temp->ncolumn+n] += temp->data[i*temp->ncolumn+n] * data_temp;
				  }
				}
			}
		}
		// ���Խ���Ԫ�ػ�Ϊ1
		for(i = 0;i<temp->nrow;i++)
		{
			data_temp = temp->data[i*temp->ncolumn+i];
			// �˴�data_temp��������0����Ϊ�Ѿ���������m������ʽֵ�ˣ������0�������������
			for(j = i;j<temp->ncolumn;j++)
			{
				temp->data[i*temp->ncolumn+j] /= data_temp;
			}
		}
		// �ٵ��ţ��Ӿ��������½����ϣ�����Ԫ�أ�ֻ���¶Խ���Ԫ��
		for(i = temp->nrow-1;i>0;i--)
		{
			for(j = i-1;j>=0;j--)
			{
				data_temp = temp->data[j*temp->ncolumn+i];
				// ͬ���ģ������Ѿ���0�ˣ���û��Ҫ�ٽ��м�����
				if(data_temp)
				{
					for(n = i;n<temp->ncolumn;n++)
					{
						temp->data[j*temp->ncolumn+n] -= data_temp*temp->data[i*temp->ncolumn+n];
					}
				}
			}
		}
		matrix_padding(temp, inversed, 0, m->ncolumn, 0);
		matrix_free(&temp);
		return 0;
	}
	else
	{
		printf("error:In det_mat (m->column != m->row)\n");
    exit(1);
	}
	return -1;
}

// ����ļ�
float matrix_trace(matrix *m)
{
	float result = 0.0f;
	int i = 0;
	if(m && m->data && (m->ncolumn==m->nrow))
	{
		for(i=0;i<m->nrow;i++)
		{
//			printf("m->data[i*m->ncolumn+i]: %f", m->data[i*m->ncolumn+i]);
			result += m->data[i*m->ncolumn+i];
		}
	}
	else
	{
		printf("null matrix pointer or not square matrix!");
		exit(1);
	}
	return result;
}

// ��ӡ���������Ա㷢��NaN
void printf_matrix(matrix *m)
{
	int i = 0;
//	printf("\r\n��ӡ�����������ֵ��������û��NaN!");
	if(m && m->data)
	{
		printf("\r\n");
		for(i = 0;i<((m->nrow)*(m->ncolumn));i++)
		{
			printf("%f  ", m->data[i]);
		}
		printf("\r\n");
	}
	else
	{
		printf("null matrix pointer!");
	}
}


