#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <cassert>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/numeric/ublas/vector.hpp>

namespace ublas = boost::numeric::ublas;

template <typename NumericType>
class KalmanFilter
{
public:
  typedef ublas::vector<NumericType> vector;
  typedef ublas::matrix<NumericType> matrix;

  KalmanFilter(size_t transitionDimension,
               size_t measurementDimension)
    : transitionModel(transitionDimension,transitionDimension),
      controlModel(transitionDimension),
      measurementModel(measurementDimension,transitionDimension)
  {}

  explicit KalmanFilter(vector X, matrix P, matrix A,
                        vector B, matrix R, matrix Q, matrix H)
    : correctedState(X),
      correctedCovarianceError(P),
      transitionModel(A),
      controlModel(B),
      measurementNoise(R),
      processNoise(Q),
      measurementModel(H)
  {}

  ~KalmanFilter()
  {}

  void initializeState(vector state, matrix covarianceError)
  {
    correctedState = state;
    correctedCovarianceError = covarianceError;
  }

  /**
   * @brief time update phase; predicts predictedState from correctedState (postState from previous iteration)
   * @return predicted vector of state - X'(k)
   */
  std::pair<vector,matrix> predict()
  {
    // predictedState (x') = transitionModel (A) * correctedState (x) + controlModel (B) * u
    predictedState = ublas::prod(transitionModel,correctedState);
    // predictedCovarianceError (P') = transitionModel (A) * correctedCovarianceError (P) * transposed (transitionModel (A)) + processNoise (Q)
    matrix transposedTrModel = ublas::trans(transitionModel);
    matrix mult = ublas::prod(correctedCovarianceError,transposedTrModel);
    predictedCovarianceError = ublas::prod(transitionModel,mult) + processNoise;
    return std::pair<KalmanFilter::vector,KalmanFilter::matrix>(predictedState,predictedCovarianceError);
  }

  /**
   * @brief time update phase; predicts predictedState from correctedState (postState from previous iteration)
   * @param u control vector
   * @return predicted vector of state - X'(k)
   */
  std::pair<vector,matrix> predict(const vector& u)
  {
    auto p = predict();
    predictedState = p.first + controlModel * u;
    return std::pair<KalmanFilter::vector,KalmanFilter::matrix>(predictedState,p.second);
  }

  /**
   * @brief measurement update phase; corrects value of predictedState basis on measurement (observation) vector.
   * @param z measurement vector
   * @return corrected vector of state - X(k)
   */
  std::pair<vector,matrix> correct(const vector& z)
  {
    // Kalman gain = P' * transposed(H) / (H * P' * transposed(H) + R)

    matrix transposedMeasurement = ublas::trans(measurementModel);
    matrix top = ublas::prod(predictedCovarianceError,transposedMeasurement);
    // bottom = H * P' * transposed(H) + R
    matrix bottom1 = ublas::prod(measurementModel, predictedCovarianceError);
    matrix bottom_all = ublas::prod(bottom1,transposedMeasurement) + measurementNoise;
    matrix bottom(bottom_all.size1(),bottom_all.size2());
    bool inverted = invertMatrix(bottom_all,bottom);
    assert(inverted);

    matrix K = ublas::prod(top,bottom); // Kalman gain

    vector residual = z - ublas::prod(measurementModel,predictedState);

    // correctedState = predictedState + kalmanGain*(z - H*x')
    correctedState = predictedState + ublas::prod(K,residual);

    // P = (I - K*H)*P'
    matrix KH = ublas::prod(K,measurementModel);
    matrix Iminus = ublas::identity_matrix<NumericType>(KH.size1(),KH.size2()) - KH;
    correctedCovarianceError = ublas::prod(Iminus,predictedCovarianceError);
    return std::pair<vector,matrix>(correctedState,correctedCovarianceError);
  }

  void setTransitionModel(matrix m)
  {
    transitionModel = m;
  }

  void setControlModel(vector v)
  {
    controlModel = v;
  }

  void setMeasurementModel(matrix m)
  {
    measurementModel = m;
  }

  void setMeasurementNoise(matrix m)
  {
    measurementNoise = m;
  }

  void setProcessNoise(matrix m)
  {
    processNoise = m;
  }

  vector getPredictedState() const
  {
    return predictedState;
  }

  vector getCorrectedState() const
  {
    return correctedState;
  }

private:

/*
 * Matrix inversion routine.
 *   Uses lu_factorize and lu_substitute in uBLAS to invert a matrix
 *
 *  found at: http://savingyoutime.wordpress.com/2009/09/21/c-matrix-inversion-boostublas/
 */
template<class T>
bool invertMatrix(const ublas::matrix<T>& input, ublas::matrix<T>& inverse)
{
  typedef ublas::permutation_matrix<std::size_t> pmatrix;

  // create a working copy of the input
  ublas::matrix<T> A(input);

  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.size1());

  // perform LU-factorization
  int res = ublas::lu_factorize(A, pm);
  if (res != 0)
    return false;

  // create identity matrix of "inverse"
  inverse.assign(ublas::identity_matrix<T> (A.size1()));

  // backsubstitute to get the inverse
  ublas::lu_substitute(A, pm, inverse);

  return true;
}

  vector predictedState; // X'(k) (a priori)
  vector correctedState; // X(k) (a posteriori)

  matrix predictedCovarianceError; // P'(k) (a priori)
  matrix correctedCovarianceError; // P(k) (a posteriori)

  matrix transitionModel; // A
  vector controlModel; // B

  matrix measurementNoise; // R
  matrix processNoise; // Q

  matrix measurementModel; // H
};

#endif // KALMANFILTER_H
