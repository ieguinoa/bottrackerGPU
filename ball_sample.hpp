#ifndef BALL_SAMPLE_H
#define BALL_SAMPLE_H

#include <opencv2/opencv.hpp>

/**
 * @brief La muestra de la pelota.
 */
class ball_sample {
public:
        /**
         * @brief Construye la muestra de la pelota.
         */
        ball_sample();
        virtual ~ball_sample();
        
        /**
         * @brief Establece el centro de la pelota.
         */
        virtual void set_center(const cv::Point &center);
        /**
         * @brief Establece la velocidad de la pelota.
         */
        virtual void set_speed(const cv::Point &speed_vector);
        /**
         * @brief Establece la forma de la pelota.
         */
        virtual void set_shape(const std::vector<cv::Point> &shape);
        
        /**
         * @brief Devuelve el centro de la pelota.
         */
        virtual cv::Point get_center() const;
        /**
         * @brief Devuelve la velocidad de la pelota.
         */
        virtual cv::Point get_speed() const;
        /**
         * @brief Devuelve la forma de la pelota.
         */
        virtual std::vector<cv::Point> get_shape() const;
        /**
         * @brief Devuelve el area de la pelota.
         */
        virtual double get_area() const;
protected:
        /**
         * @brief Mantiene el centro de la pelota.
         */
        cv::Point center;
        /**
         * @brief Mantiene la velocidad de la pelota.
         */
        cv::Point speed;
        /**
         * @brief Mantiene la forma de la pelota.
         */
        std::vector<cv::Point> shape;
        /**
         * @brief Mantiene el area de la pelota.
         */
        double area;
};

#endif /* BALL_SAMPLE_H */
