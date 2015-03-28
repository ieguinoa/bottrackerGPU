#include "ball_sample.hpp"
#include "utils.hpp"

ball_sample::ball_sample()
: area(0.0)
{
}

ball_sample::~ball_sample()
{
}

void ball_sample::set_center(const cv::Point &center)
{
        this->center = center;
}
void ball_sample::set_speed(const cv::Point &speed_vector)
{
        this->speed = speed_vector;
}
void ball_sample::set_shape(const std::vector<cv::Point> &shape)
{
        this->shape.assign(shape.begin(), shape.end());
        this->area = polygon_area(shape);
}

cv::Point ball_sample::get_center() const
{
        return center;
}
cv::Point ball_sample::get_speed() const
{
        return speed;
}
std::vector<cv::Point> ball_sample::get_shape() const
{
        return shape;
}

double ball_sample::get_area() const
{
        return area;
}
