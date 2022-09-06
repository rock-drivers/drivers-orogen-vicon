#ifndef VICON_UNCERTAINTY_HPP
#define VICON_UNCERTAINTY_HPP

#include <Eigen/Core>
#include <Eigen/Cholesky>

/** Boost **/
#include <boost/circular_buffer.hpp> /** For circular buffers **/

namespace vicon {
    template <typename _EigenMatrixType>
    class ViconUncertainty
    {
        protected:
            size_t dimension;
            boost::circular_buffer< _EigenMatrixType > data;


        public:
            ViconUncertainty(size_t dimension = 0)
            {
                this->dimension = dimension;
                data = boost::circular_buffer< _EigenMatrixType >(this->dimension);
            }


            void push(const _EigenMatrixType& value)
            {
                if (this->dimension > 0)
                {
                    data.push_front(value);
                }
            }


            _EigenMatrixType getMean()
            {
                _EigenMatrixType sum;
                sum.setZero();

                if (this->dimension > 0)
                {
                    for(register unsigned int i=0; i<data.size(); ++i)
                    {
                        sum += data[i];
                    }
                }
                return sum/data.size();
            }

            _EigenMatrixType getVariance()
            {
                _EigenMatrixType mean = getMean();
                _EigenMatrixType temp;
                temp.setZero();

                if (this->dimension > 0)
                {
                    for(register unsigned int i=0; i<data.size(); ++i)
                    {
                        temp += (mean - data[i]) * (mean - data[i]);
                    }

                }
                return temp/data.size();
            }

            _EigenMatrixType getStdDev()
            {
                _EigenMatrixType variance = getVariance();
                Eigen::LLT< _EigenMatrixType > lltOfVar (variance);

                return variance.matrixL();
            }

            inline size_t size()
            {
                return data.size();
            }

            inline size_t capacity()
            {
                return data.capacity();
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif
