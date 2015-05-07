#ifndef HECTOR_QUADROTOR_CONTROLLER_FILTERS_H
#define HECTOR_QUADROTOR_CONTROLLER_FILTERS_H

#include <boost/circular_buffer.hpp>
#include <vector>

namespace hector_quadrotor_controller
{

  class Filter
  {

  private:
    std::vector<double> params_;

  public:
    Filter() { }

//    Filter(std::vector<double> params)
//    : params_(params){ }

//    void setParams(std::vector<double> params){
//      params_ = params;
//    }

    virtual ~Filter() { }

    virtual double filter(double value) = 0;

  };

  class PassThroughFilter : public Filter
  {

  public:
    PassThroughFilter() { }

    virtual ~PassThroughFilter() { }

    virtual double filter(double value){
      return value;
    }
  };


  class ButterworthFilter : public Filter
  {

  private:

    // TODO allow clients to parametrize filter. coefficient array/vector?
    static const int zeros_ = 2;
    static const int poles_ = 2;
    static const double gain_ = 1.482463775e+01;
    static const double a_ = -0.4128015981;
    static const double b_ = 1.1429805025;

    boost::circular_buffer <double> input_buffer_, output_buffer_;

  public:

    ButterworthFilter() : Filter(), input_buffer_(zeros_ + 1), output_buffer_(poles_ + 1)
    {
    }

    virtual ~ButterworthFilter()
    {
    }

    double filter(double value)
    {
      input_buffer_.push_back(value / gain_);

      if (output_buffer_.size() == output_buffer_.capacity())
      {
        output_buffer_.push_back(input_buffer_[0] + input_buffer_[2] + 2 * input_buffer_[1]
                                 + a_ * output_buffer_[1] + b_ * output_buffer_[2]);
        value = output_buffer_[2];

      }
      else if (input_buffer_.size() == input_buffer_.capacity())
      {
        output_buffer_.push_back((input_buffer_[0] + input_buffer_[2]) + 2 * input_buffer_[1]);
      }

      return value;

    }
  };


}
#endif //HECTOR_QUADROTOR_CONTROLLER_FILTERS_H
