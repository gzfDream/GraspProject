#pragma once
/*************************************************************************
����˵���� ��ƽ���ϵ�һЩ�е������С���˵���Բ��ϣ���������ֵ�ֽⷨ
�����С���˽���Ϊ��Բ������
������ʽ�� cvFitEllipse2f��arrayx,arrayy,box����
����˵���� arrayx: arrayx[n],ÿ��ֵΪx��һ����
arrayx: arrayy[n],ÿ��ֵΪy��һ����
n     : ��ĸ���
box   : box[5],��Բ������������ֱ�Ϊcenter.x,center.y,2a,2b,xtheta
esp: �⾫�ȣ�ͨ��ȡ1e-6,����ǽⷽ���õ�˵
***************************************************************************/
#include<cstdlib>
#include<float.h>
#include<vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
using namespace std;

class LSEllipse
{
public:
	LSEllipse(void);
	~LSEllipse(void);
	vector<float> fittingEllipse_mine(vector<Eigen::Vector2f> vec_point);
	vector<float> getEllipseparGauss(vector<Eigen::Vector2f> vec_point);
	void cvFitEllipse2f(float *arrayx, float *arrayy, int n, float *box);
private:
	int SVD(float *a, int m, int n, float b[], float x[], float esp);
	int gmiv(float a[], int m, int n, float b[], float x[], float aa[], float eps, float u[], float v[], int ka);
	int ginv(float a[], int m, int n, float aa[], float eps, float u[], float v[], int ka);
	int muav(float a[], int m, int n, float u[], float v[], float eps, int ka);
};
