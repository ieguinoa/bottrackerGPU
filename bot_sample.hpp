#ifndef BOT_SAMPLE_H
#define BOT_SAMPLE_H

#include "types.hpp"

#include <opencv2/opencv.hpp>

/**
 * @brief La muestra de un robot.
 */
class bot_sample {
public:
        /**
         * @brief Construye una muestra para el robot con id dado.
         */
        bot_sample(e_robot id);
        virtual ~bot_sample();
        
        /**
         * @brief Establece el centro del robot.
         */
        virtual void set_center(const cv::Point &center);
        /**
         * @brief Establece la orientacion del robot.
         */
        virtual void set_orientation(const cv::Point &orient_vector);
        /**
         * @brief Establece la velocidad del robot.
         */
        virtual void set_speed(const cv::Point &speed_vector);
        /**
         * @brief Establece la forma del robot.
         */
        virtual void set_shape(const std::vector<cv::Point> &shape);
        
        /**
         * @brief Devuelve el identificador del robot.
         */
        virtual e_robot get_id() const;
        /**
         * @brief Devuelve el centro del robot.
         */
        virtual cv::Point get_center() const;
        /**
         * @brief Devuelve la orientacion del robot.
         */
        virtual cv::Point get_orientation() const;
        /**
         * @brief Devuelve la velocidad del robot.
         */
        virtual cv::Point get_speed() const;
        /**
         * @brief Devuelve la forma del robot.
         */
        virtual std::vector<cv::Point> get_shape() const;
        /**
         * @brief Devuelve el area del robot.
         */
        virtual double get_area() const;
protected:
        /**
         * @brief Mantiene el centro del robot.
         */
        cv::Point center;
        /**
         * @brief Mantiene la orientacion del robot.
         */
        cv::Point orientation;
        /**
         * @brief Mantiene la velocidad del robot.
         */
        cv::Point speed;
        /**
         * @brief Mantiene la forma del robot.
         */
        std::vector<cv::Point> shape;
        /**
         * @brief Mantiene el area del robot.
         */
        double area;
        /**
         * @brief Mantiene el identificador del robot.
         */
        e_robot id;
};

#endif /* BOT_SAMPLE_H */
