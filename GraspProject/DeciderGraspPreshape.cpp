#include "DeciderGraspPreshape.h"


DeciderGraspPreshape::DeciderGraspPreshape()
{
	shape_threshold = 1.2;

	minThicknessPower = 0.020f;
	maxThicknessPower = 0.060f;
	minLengthPower = 0.020f;

	minThicknessPrecision = 0.001f;
	maxThicknessPrecision = 0.020f;
	minLengthPrecision = 0.0010f;
	maxLengthPrecision = 0.0300f;
}


DeciderGraspPreshape::~DeciderGraspPreshape()
{
}


bool DeciderGraspPreshape::calculatePCShape_isRound(float lam1, float lam2) {
	if (lam2 != 0)
		if ((lam1 / lam2) <= shape_threshold)
			return true;
		else
			return false;
	else
		return false;
}


void DeciderGraspPreshape::setShapethreshold(float threshold) {
	shape_threshold = threshold;
}


void DeciderGraspPreshape::setLengthPrecision(float min, float max) {
	minLengthPrecision = min;
	maxLengthPrecision = max;
}


void DeciderGraspPreshape::setLengthPower(float min) {
	minLengthPower = min;
}


void DeciderGraspPreshape::setThicknessPrecision(float minThickness, float maxThickness)
{
	minThicknessPrecision = minThickness;
	maxThicknessPrecision = maxThickness;
}


void DeciderGraspPreshape::setThicknessPower(float minThickness, float maxThickness)
{
	minThicknessPower = minThickness;
	maxThicknessPower = maxThickness;
}


bool DeciderGraspPreshape::decidePowerPreshape(float length, float thickness, bool isRound)
{
	if (isRound) {
		if ((thickness >= minThicknessPower && thickness <= maxThicknessPower))
		{
			return true;
		}
	}
	else{
		if ((thickness >= minThicknessPower && thickness <= maxThicknessPower) || length >= minLengthPower)
		{
			return true;
		}
	}

	return false;
}


bool DeciderGraspPreshape::decidePrecisionPreshape(float length, float thickness, bool isRound)
{

	if (isRound){
		if (thickness > minThicknessPrecision && thickness < maxThicknessPrecision /*&& length > minLengthPrecision && length < maxLengthPrecision*/) {
			return true;
		}
	}
	else{
		if (thickness > minThicknessPrecision && thickness < maxThicknessPrecision && length > minLengthPrecision && length < maxLengthPrecision) {
			return true;
		}
	}

	

	return false;
}