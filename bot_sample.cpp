#include "bot_sample.hpp"
#include "utils.hpp"

bot_sample::bot_sample(e_robot id)
: area(0.0)
, id(id)
{
}

bot_sample::~bot_sample()
{
}

void bot_sample::set_center(const cv::Point &center)
{
        this->center = center;
}
void bot_sample::set_orientation(const cv::Point &orient_vector)
{
        this->orientation = orient_vector;
}
void bot_sample::set_speed(const cv::Point &speed_vector)
{
        this->speed = speed_vector;
}
void bot_sample::set_shape(const std::vector<cv::Point> &shape)
{
        this->shape.assign(shape.begin(), shape.end());
        this->area = polygon_area(shape);
}

e_robot bot_sample::get_id() const
{
        return id;
}
cv::Point bot_sample::get_center() const
{
        return center;
}
cv::Point bot_sample::get_orientation() const
{
        return orientation;
}
cv::Point bot_sample::get_speed() const
{
        return speed;
}
std::vector<cv::Point> bot_sample::get_shape() const
{
        return shape;
}

double bot_sample::get_area() const
{
        return area;
}
