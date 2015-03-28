/**
 * @file utils.hpp
 * @brief Utilidades varias usadas en bottracker
 */
#ifndef UTILS_HPP
#define UTILS_HPP

#include "types.hpp"

#include <stdint.h>
#include <opencv2/opencv.hpp>

/**
 * @brief Tipo usado para representar nanosegundos.
 */
typedef uint64_t nsecs_t;

/**
 * @brief Convierte nanosegundos a nanosegundos.
 */
#define N2NSECS(t) (t)

/**
 * @brief Convierte microsegundos a nanosegundos.
 */
#define U2NSECS(t) ((t)*1000)

/**
 * @brief Convierte milisegundos a nanosegundos.
 */
#define M2NSECS(t) ((t)*1000000)

/**
 * @brief Convierte segundos a nanosegundos.
 */
#define S2NSECS(t) ((t)*1000000000)

/**
 * @brief Convierte nanosegundos a microsegundos.
 */
#define N2USECS(t) ((t)/1000)

/**
 * @brief Convierte nanosegundos a milisegundos.
 */
#define N2MSECS(t) ((t)/1000000)

/**
 * @brief Convierte nanosegundos a segundos.
 */
#define N2SSECS(t) ((t)/1000000000)

/**
 * @brief Retorna el timestamp actual en nanosegundos.
 */
nsecs_t get_timestamp();

/**
 * @brief Calcula si el punto p2 esta a la derecha
 * de la linea entre p0 y p1.
 */
bool is_right(const cv::Point &p0, const cv::Point &p1, const cv::Point &p2);

/**
 * @brief Calcula si el punto p2 esta a la izquierda
 * de la linea entre p0 y p1.
 */
bool is_left (const cv::Point &p0, const cv::Point &p1, const cv::Point &p2);

/**
 * @brief Calcula si el punto p esta en un area delimitada
 * por (0,0) y size.
 */
bool in_bounds(const cv::Point &p, const cv::Size &size);

/**
 * @brief Calcula el modulo (largo) de un punto.
 */
double module(const cv::Point &p0);

/**
 * @brief Calcula la distancia entre dos puntos.
 */
double p_distance(const cv::Point &p0, const cv::Point &p1);

/**
 * @brief Devuelve true si el valor esta a lo sumo max_error
 * del esperado.
 */
bool approx(double value, double expected, double max_error);

/**
 * @brief Calcula el area de un cuadrilatero formado por los
 * 4 puntos dados.
 */
double polygon_area(const cv::Point &p0, const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);

/**
 * @brief Calcula el area de un poligono definido por los puntos dados.
 */
double polygon_area(const std::vector<cv::Point> &points);

/**
 * @brief Retorna los primeros 4 puntos de un vector de puntos.
 * @param points Un vector con al menos 4 puntos.
 */
void vec2pts (const std::vector<cv::Point> &points, cv::Point &p0, cv::Point &p1, cv::Point &p2, cv::Point &p3);

/**
 * @brief Agrega los 4 puntos a un vector.
 */
void pts2vec (std::vector<cv::Point> &points, const cv::Point &p0, const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);

/**
 * @brief Normaliza componentes BGR de color al intervalo [0.0, 1.0]
 */
void normalize_color (double &c0, double &c1, double &c2);

/**
 * @brief Devuelve en name la identidad del robot dado.
 */
void get_robot_name (e_robot robot_id, std::string &name);

#endif /* UTILS_HPP */
