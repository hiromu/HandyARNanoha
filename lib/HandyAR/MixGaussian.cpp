#include <HandyAR/MixGaussian.h>
#include <stdio.h>

MixGaussian::MixGaussian()
{
	_nMixture = 0;
	_nDim = 0;
	for(int i = 0; i < MAX_NUM_GAUSSIAN ; i++) {
		_MeanMat[i] = 0;
		_CovMat [i] = 0;
		_CovMatI[i] = 0;
	}
}

MixGaussian::~MixGaussian()
{
	for(int i = 0; i < _nMixture ; i++) {
		if(_MeanMat[i]) cvReleaseMat(&_MeanMat[i]);
		if(_CovMat [i]) cvReleaseMat(&_CovMat [i]);
		if(_CovMatI[i]) cvReleaseMat(&_CovMatI[i]);
	}
}

bool MixGaussian::LoadFile(char *filename)
{
	FILE *fp = fopen(filename, "rt");
	if(!fp) return false;

	int nMixture = 0;
	fscanf(fp, "%d", &nMixture);

	int nDim = 0;
	fscanf(fp, "%d", &nDim);
	_nDim = nDim;

	CvMat *MeanMat = cvCreateMat(nDim, 1, CV_64FC1);
	CvMat *CovMat = cvCreateMat(nDim, nDim, CV_64FC1);
	cvSetZero(CovMat);
	float Weight = 0;

	for(int i = 0; i < nMixture ; i++) {
		float value;
		// Set Mean
		for(int j = 0; j < nDim ; j++) {
			fscanf(fp, "%f", &value);
			cvSetReal1D(MeanMat, j, value);
		}
		// Set diagonal Covariance
		for(int j = 0; j < nDim ; j++) {
			fscanf(fp, "%f", &value);
			cvSetReal2D(CovMat, j, j, value);
		}
		// Set Weight
		fscanf(fp, "%f", &value);
		Weight = value;

		// add a gaussian
		AddGaussian(MeanMat, CovMat, Weight);
	}
	cvReleaseMat(&MeanMat);
	cvReleaseMat(&CovMat);

	fclose(fp);
	return true;
}

bool MixGaussian::AddGaussian(
		CvMat * MeanMat,
		CvMat * CovMat,
		float Weight)
{
	if(_nMixture == MAX_NUM_GAUSSIAN)
		return false;

	_MeanMat[_nMixture] = cvCloneMat(MeanMat);
	_CovMat [_nMixture] = cvCloneMat(CovMat);
	_CovMatI[_nMixture] = cvCloneMat(CovMat);
	cvInvert(CovMat, _CovMatI[_nMixture]);
	_Weight [_nMixture] = Weight;

	_nMixture++;

	return true;
}

float MixGaussian::GetProbability(CvMat * Sample)
{
	double P = 0.0;
	CvMat *diffMat = cvCloneMat(Sample);
	CvMat *diffMatT = cvCreateMat(1, _nDim, CV_64FC1);
	double expo;
	CvMat expoMat = cvMat(1, 1, CV_64FC1, &expo);

	for(int k = 0; k < _nMixture ; k++) {
		cvSub(Sample, _MeanMat[k], diffMat);
		cvTranspose(diffMat, diffMatT);
		cvMatMul(_CovMatI[k], diffMat, diffMat);
		cvMatMul(diffMatT, diffMat, &expoMat);
		expo *= (-0.5);
		P += (_Weight[k] * 1.0 / (pow(2 * CV_PI, 1.5) * sqrt(cvDet(_CovMat[k]))) * exp(expo));
	}

	cvReleaseMat(&diffMat);
	cvReleaseMat(&diffMatT);

	return P;
}

void MixGaussian::MakeLookUpTable()
{
	CvMat * SampleMat = cvCreateMat(3, 1, CV_64FC1);

	for(int R = 0; R < 256 ; R++) {
		printf(".");
		for(int G = 0; G < 256 ; G++) {
			for(int B = 0; B < 256 ; B++) {
				cvSetReal1D(SampleMat, 0, (double)R);
				cvSetReal1D(SampleMat, 1, (double)G);
				cvSetReal1D(SampleMat, 2, (double)B);

				_Probability[R][G][B] = GetProbability(SampleMat);
			}
		}
	}

	cvReleaseMat(&SampleMat);
}

float MixGaussian::GetProbabilityByLookup(int R, int G, int B)
{
	return _Probability[R][G][B];
}

bool MixGaussian::SaveLookUpTable(char *filename)
{
	FILE *fp = fopen(filename, "wb");
	if(!fp)
		return false;

	for(int R = 0; R < 256 ; R++)
		for(int G = 0; G < 256 ; G++)
			fwrite(_Probability[R][G], sizeof(float), 256, fp);

	fclose(fp);
	return true;
}

bool MixGaussian::LoadLookUpTable(char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if(!fp)
		return false;

	for(int R = 0; R < 256 ; R++) {
		for(int G = 0; G < 256 ; G++) {
			if(fread(_Probability[R][G], sizeof(float), 256, fp) != 256) {
				fclose(fp);
				return false;
			}
		}
	}

	fclose(fp);
	return true;
}
