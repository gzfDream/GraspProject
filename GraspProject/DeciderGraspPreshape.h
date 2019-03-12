#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>



class DeciderGraspPreshape
{
public:
	DeciderGraspPreshape();
	~DeciderGraspPreshape();

	//!	判断是否是圆形，是返回true 
	bool calculatePCShape_isRound(float lam1, float lam2);
	void setShapethreshold(float threshold);

	void setLengthPrecision(float min, float max);
	void setLengthPower(float min);
	void setThicknessPrecision(float minThickness, float maxThickness);
	void setThicknessPower(float minThickness, float maxThickness);
	bool decidePowerPreshape(float length, float thickness, bool isRound);
	bool decidePrecisionPreshape(float length, float thickness, bool isRound);

protected:
	float shape_threshold;	//形状阈值

	float minThicknessPrecision;
	float maxThicknessPrecision;
	float minLengthPrecision;
	float maxLengthPrecision;

	float minThicknessPower;
	float maxThicknessPower;
	float minLengthPower;
};

