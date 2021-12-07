#include "matrix.h"
// 矩阵的基本操作方法
// 矩阵是按照行优先的原则来存储的，意即一行一行的存储的
// 设置矩阵的内容，成功返回0，失败返回-1，默认输入参数m是已经初始化好了的矩阵
int matrix_set_data(matrix *m, float *val, u32 list_size)
{
	int i = 0;
	if(m && m->data && val)
	{
		// sizeof只能用于数组，才能显示出大小，若只是用于指针，则只会显示这个指针是多少字节
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

// 向量归一化，可以是行向量，也可以是列向量，成功返回0，失败返回-1
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

// 快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	//把float转为long，i是这样得来的：将原为float的x的所有字节以long型读取
	long i = *(long*)&y;
	//0x5f3759df减去i的一半
	i = 0x5f3759df - (i>>1);
	//把long转为float，新的y是这样得来的：将原为long的i的所有字节以float型读取
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// 创建单位阵，成功则返回分配好内存的matrix结构体指针，失败则返回空指针
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

// 将已有矩阵的内容全部置0，成功返回0，失败返回-1，
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

// 矩阵乘标量，成功则返回0，否则返回-1
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

// 矩阵初始化，若成功，则返回一个matrix结构体指针，失败则返回空指针
matrix* matrix_init(u32 num_row, u32 num_column)
{
	matrix *result = NULL;
	result = (matrix*)mymalloc(SRAMIN, sizeof(matrix));
	// 不允许出现行或者列数目为0的情况
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

// 释放矩阵内存
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

// 在矩阵某个位置写入值，成功则返回0，失败则返回-1
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

// 读取矩阵某个位置的数据并返回，此处，若是出现错误，返回何值？
// 还是直接结束程序？
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

// 以输入的数组中的数据为基础构建对角矩阵
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


// 矩阵的基本运算方法

// 矩阵加，成功则返回0，否则返回-1
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

// 矩阵减，成功则返回0，否则返回-1
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

// 矩阵乘，成功则返回0，否则返回-1
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

// 矩阵转置，m是已经初始化好了的矩阵，若成功则返回matrix结构体指针，否则返回空指针
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

// 矩阵求逆
int matrix_inverse(matrix *m, matrix *inversed)
{
	float temp_det = 0.0f;
	if(m && inversed)
	{
		temp_det = matrix_determinant(m);
		// 当行列式的值不为0的时候，才可以计算逆矩阵
		if(temp_det)
		{
			matrix_adjoint(m, inversed);
			// 将伴随矩阵除以行列式值，即得到逆矩阵
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

// 求方阵的行列式值
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

// 求方阵的行列式值(以数组作为输入，有递归)
float list_determinant(float *list, u32 number)
{
	float result = 0.0f;
	int i = 0;
	float *temp = mymalloc(SRAMIN, sizeof(float)*(number-1)*(number-1));
	
	// 若矩阵大小不是1*1的，则进行递归
	if(temp && list)
	{
		if(number == 1)
	  {
		  result += list[0];
	  }
		else
		{
			// 从第一行开始展开，计算各个子行列式的值
			for(i=0;i<number;i++)
			{
				// 若为0，则其子行列式就不必计算了
				if(list[i])
				{
					get_sub_square_matrix(list, temp, number, 0, i);
					// 得到子矩阵之后，进行递归
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
    // 记得释放内存
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

// 求方阵的子方阵(去除了某行以及某列的元素)
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

// 求伴随矩阵
int matrix_adjoint(matrix *m, matrix *m_star)
{
	float temp_det = 0.0f;
	float* temp = NULL;
	int i = 0;
	int j = 0;
	// 如果这个矩阵的元素个数为1，则直接返回伴随矩阵
	// 必须是方阵，才能求伴随矩阵
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
				// 计算余子式
				temp_det = list_determinant(temp, m->nrow-1);
				// 计算代数余子式，顺便把结果矩阵进行转置
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

// 修改大矩阵或者是从大矩阵中取小矩阵，
// 如果choose为1, 则在原矩阵的某个位置填入小矩阵的数据进去，以得到新的大矩阵，成功则返回0，失败则返回-1
// 如果choose为0, 则在原矩阵的某个位置取出数据填充到小矩阵里，以得到新的小矩阵，成功则返回0，失败则返回-1
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

// 3阶向量叉乘，a*(叉乘)b=c
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

// 求方阵的行列式值，此处采用的是行变换（列主元消去法）的方法，将原矩阵变为上三角矩阵，再求行列式值
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
			// 从第一列开始，首先寻找这一列绝对值最大的那个值，并且记录它所在的行的行号，
			// 然后将现在的第i行和这记录的行号这行交换位置
			max = fabsf(temp->data[i*(temp->ncolumn)+i]);
			max_row = i;
			// 选择绝对值最大的，是为了提高算法的稳定性，很多时候，不影响计算结果
			// 如果选的主元特别小，那么后面再进行除的时候，就会把矩阵中的元素变得特别大，舍入误差扩大
			// 尽量降低矩阵中的元素的改变的幅度
			for(j = i+1;j<temp->nrow;j++)
			{
				// 下面的i表示第几列（因为前面的i既表示列，又表示行，前面迭代的是对角线上的元素）
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
			// 把第i行和最大值所在的那行交换位置
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
			// 把第i行以下的所有行的第i列，都变为0
			for(j = i+1;j<temp->nrow;j++)
			{
				// 若已经为0，就没必要再进行计算了
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

// 矩阵求逆，此处采用的是Gauss-Jordan列主元消去法，成功则返回0，否则返回-1
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
		// 首先在逆矩阵中填入1，形成单位阵
		for(i = 0;i<inversed->nrow;i++)
		{
			inversed->data[i*inversed->ncolumn+i] = 1.0f;
		}
		temp = matrix_init(m->nrow, m->ncolumn+inversed->ncolumn);
		matrix_padding(temp, m, 0, 0, 1);
		matrix_padding(temp, inversed, 0, m->ncolumn, 1);
		// 先把矩阵化为上三角矩阵
		for(i = 0;i<temp->nrow-1;i++)
		{
			// 从第一列开始，首先寻找这一列绝对值最大的那个值，并且记录它所在的行的行号，
			// 然后将现在的第i行和这记录的行号这行交换位置
			max = fabsf(temp->data[i*(temp->ncolumn)+i]);
			max_row = i;
			for(j = i+1;j<temp->nrow;j++)
			{
				// 下面的i表示第几列（因为前面的i既表示列，又表示行，前面迭代的是对角线上的元素）
				if(max<fabsf(temp->data[j*(temp->ncolumn)+i]))
				{
					max = fabsf(temp->data[j*(temp->ncolumn)+i]);
					max_row = j;
				}
			}
			// 把第i行和最大值所在的那行交换位置
			if(i != max_row)
			{
				for(j = i;j<temp->ncolumn;j++)
				{
					data_temp = temp->data[i*temp->ncolumn+j];
					temp->data[i*temp->ncolumn+j] = temp->data[max_row*temp->ncolumn+j];
					temp->data[max_row*temp->ncolumn+j] = data_temp;
				}
			}
			// 把第i行以下的所有行的第i列，都变为0
			for(j = i+1;j<temp->nrow;j++)
			{
				// 若已经为0，就没必要再进行计算了
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
		// 将对角线元素化为1
		for(i = 0;i<temp->nrow;i++)
		{
			data_temp = temp->data[i*temp->ncolumn+i];
			// 此处data_temp不可能是0，因为已经检查过矩阵m的行列式值了，如果是0，就求不了逆矩阵
			for(j = i;j<temp->ncolumn;j++)
			{
				temp->data[i*temp->ncolumn+j] /= data_temp;
			}
		}
		// 再倒着，从矩阵最右下角往上，消除元素，只留下对角线元素
		for(i = temp->nrow-1;i>0;i--)
		{
			for(j = i-1;j>=0;j--)
			{
				data_temp = temp->data[j*temp->ncolumn+i];
				// 同样的，若是已经是0了，就没必要再进行计算了
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

// 求方阵的迹
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

// 打印整个矩阵，以便发现NaN
void printf_matrix(matrix *m)
{
	int i = 0;
//	printf("\r\n打印出矩阵的所有值，看看有没有NaN!");
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


