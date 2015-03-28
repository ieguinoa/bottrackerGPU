#include "utils.hpp"

#include <time.h>

using cv::Point;

nsecs_t get_timestamp()
{
        struct timespec ts;
        nsecs_t retval;
        
        clock_gettime(CLOCK_MONOTONIC, &ts);
        retval = S2NSECS(ts.tv_sec) + ts.tv_nsec;
        return retval;
}

bool is_right(const Point &p0, const Point &p1, const Point &p2)
{
        Point pd = p1-p0;
        Point dpT = Point(-pd.y, pd.x);
        
        double perpdot = dpT.dot(p2-p1);
        return perpdot > 0.0;
}

bool is_left(const Point &p0, const Point &p1, const Point &p2)
{
        return !is_right(p0, p1, p2);
}

bool in_bounds(const cv::Point& p, const cv::Size& size)
{
        return p.x >= 0 && p.x < size.width && p.y >= 0 && p.y < size.height;
}


double module(const cv::Point& p0)
{
        return sqrt((double)p0.dot(p0));
}

double p_distance(const cv::Point& p0, const cv::Point& p1)
{
        return module(p1-p0);
}

bool approx(double value, double expected, double max_error)
{
        return fabs(expected-value) < max_error;
}

double polygon_area(const cv::Point& a, const cv::Point& b, const cv::Point& c, const cv::Point& d)
{
        double area = 0;
        area += a.x * b.y;
        area += b.x * c.y;
        area += c.x * d.y;
        area += d.x * a.y;
        area -= a.y * b.x;
        area -= b.y * c.x;
        area -= c.y * d.x;
        area -= d.y * a.x;
        
        area /= 2;
        return(area < 0 ? -area : area);
}

double polygon_area(const std::vector< Point >& points)
{
        double area = 0;
        int npoints = points.size();
        for (int i = 0; i < npoints; ++i) {
                int j = (i+1)%npoints;
                area += (points[i].x*points[j].y);
                area -= (points[i].y*points[j].x);
        }
        area /= 2;
        return (area < 0 ? -area : area);
}

void vec2pts (const std::vector<cv::Point> &points, cv::Point &p0, cv::Point &p1, cv::Point &p2, cv::Point &p3)
{
        p0 = points[0];
        p1 = points[1];
        p2 = points[2];
        p3 = points[3];
}

void pts2vec (std::vector<cv::Point> &points, const cv::Point &p0, const cv::Point &p1, const cv::Point &p2, const cv::Point &p3)
{
        points.push_back(p0);
        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
}

void normalize_color (double &c0, double &c1, double &c2) {
        double max = c0;
        if (c1 > max) max = c1;
        if (c2 > max) max = c2;
        
        c0 /= max;
        c1 /= max;
        c2 /= max;
}

void get_robot_name(e_robot robot_id, std::string& name)
{
        switch (robot_id) {
                case LB_0: name = "B0"; break;
                case LB_1: name = "B1"; break;
                case LB_2: name = "B2"; break;
                case LB_3: name = "B3"; break;
                case LB_4: name = "B4"; break;
                case YY_0: name = "Y0"; break;
                case YY_1: name = "Y1"; break;
                case YY_2: name = "Y2"; break;
                case YY_3: name = "Y3"; break;
                case YY_4: name = "Y4"; break;
                default: name = "??"; break;
        }
}

