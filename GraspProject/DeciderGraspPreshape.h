#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>



class DeciderGraspPreshape
{
public:
	DeciderGraspPreshape();
	~DeciderGraspPreshape();

	//!	�ж��Ƿ���Բ�Σ��Ƿ���true 
	bool calculatePCShape_isRound(float lam1, float lam2);
	void setShapethreshold(float threshold);

	void setLengthPrecision(float min, float max);
	void setLengthPower(float min);
	void setThicknessPrecision(float minThickness, float maxThickness);
	void setThicknessPower(float minThickness, float maxThickness);
	bool decidePowerPreshape(float length, float thickness, bool isRound);
	bool decidePrecisionPreshape(float length, float thickness, bool isRound);

protected:
	float shape_threshold;	//��״��ֵ

	float minThicknessPrecision;
	float maxThicknessPrecision;
	float minLengthPrecision;
	float maxLengthPrecision;

	float minThicknessPower;
	float maxThicknessPower;
	float minLengthPower;
};

