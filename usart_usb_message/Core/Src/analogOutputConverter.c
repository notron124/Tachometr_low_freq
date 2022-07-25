#include "analogOutputConverter.h"

double analogOutputConverter(float freq, float XMIN, float XMAX, float YMIN, float YMAX)
{
	return ((freq - XMIN)*(YMAX - YMIN)) / (XMAX - XMIN) + YMIN;
}
