#include <stdint.h>

typedef struct _SignalStatistics
{
	// double is required to calculate a numerically stable variance
	double sum;
	double sum2;
	uint32_t numSamples;

	// Following values are updated with SignalStatisticsUpdate
	double mean;
	double variance;
	double standardDeviation;
} SignalStatistics;

void SignalStatisticsReset(SignalStatistics* pStat);
void SignalStatisticsSample(SignalStatistics* pStat, double sample);
double SignalStatisticsGetVariance(const SignalStatistics* pStat);
double SignalStatisticsGetMean(const SignalStatistics* pStat);
void SignalStatisticsUpdate(SignalStatistics* pStat);