#ifndef HECTOR_QUADROTOR_CONTROLLER_FILTERS_H
#define HECTOR_QUADROTOR_CONTROLLER_FILTERS_H

#include <boost/circular_buffer.hpp>
#include <vector>

namespace hector_quadrotor_controller
{

  class ButterworthFilter
  {

  private:

    // http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
    // Butterworth LPF 2nd order, 100 Hz sample rate, 5 Hz corner frequency
    static const double gain_ = 4.979245121e+01;
    static const double a_ = -0.6413515381;
    static const double b_ = 1.5610180758;

    boost::circular_buffer <double> input_buffer_, output_buffer_;

  public:

    ButterworthFilter() : input_buffer_(3), output_buffer_(3)
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
