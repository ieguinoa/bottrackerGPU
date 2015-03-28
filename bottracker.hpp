/**
 * @file bottracker.hpp
 * @brief Define una API para la deteccion de robots
 * en un juego de futbol de robots.
 */
#ifndef BOTTRACKER_H
#define BOTTRACKET_H

/**
 * @mainpage
 * @section intro_section Introduccion
 * bottracker es una libreria que usa opencv para procesar
 * imagenes en un partido de futbol de robots.
 *
 * bottracker trabaja en partidos de 5 jugadores por equipo,
 * con una pelota de color naranja.
 * Cada equipo se diferencia por el uso de una "camiseta":
 * un equipo lleva el color amarillo, mientras que el otro
 * lleva color celeste.
 *
 * bottracker trabaja identificando cada uno de los robots
 * de cada equipo usando un sistema de colores: dado
 * el color principal del equipo (Amarillo o celeste), los
 * robots toman identificadores con una combinacion del color
 * principal y dos colores mas: violeta y verde.
 *
 * Mirados desde arriba, los robots se segmentan en
 * 4 cuadrados y los colores de cada cuadrado se leen en
 * sentido contrario de las agujas del reloj.
 * Los patrones de colores son, usando key como color principal:
 * 
 * robot 0: KEY, KEY, verde, verde.
 * 
 * robot 1: KEY, KEY, violeta, violeta.
 * 
 * robot 2: KEY, KEY, verde, violeta.
 * 
 * robot 3: KEY, KEY, violeta, verde.
 * 
 * robot 4: KEY, violeta, KEY, verde.
 */
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>


#include "field_sample.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <map>
#include <vector>
#include <string>
#include <list>

/**
 * @brief bot_tracker asiste en la detección de robots para
 * un juego de futbol de robots.
 */
class bot_tracker {
public:
        /**
         * @brief Constructor.
         * @param background La imagen del fondo sin los robots.
         */
        bot_tracker (cv::Mat &background, int history_size);

        /**
         * @brief Destructor.
         */
        virtual ~bot_tracker();

        /**
         * @brief Establece el valor de umbral para separar
         * los robots del fondo.
         */
        void set_threshold (double threshold);

        /**
         * @brief Establece el umbral de certeza para el color rojo.
         */
        void set_redcolor_threshold(double threshold);
        
        /**
         * @brief Establece la muestra de color para un color dado.
         * Usado para calibración.
         */
        void set_color_code(e_color color_id, double blue, double green, double red);

        /**
         * @brief Establece los limites de area para
         * la deteccion de un robot.
         * Se intentara detectar un robot solo si el
         * area del poligono esta en el rango [min, max]
         */
        void set_bot_area_limits(double min, double max);
        
        /**
         * @brief Establece los limites de area para
         * la deteccion de la pelota.
         * Se intentara detectar la pelota solo si el
         * area del poligono esta en el rango [min, max]
         */
        void set_ball_area_limits(double min, double max);

        /**
         * @brief Procesa el frame buscando los robots y la pelota.
         * @return El tiempo de procesamiento del frame en nanosegundos.
         */
        virtual nsecs_t process (cv::Mat &frame, cv::gpu::GpuMat &d_frame);

        /**
         * @brief Devuelve una muestra del campo.
         * @param sample_ptr Una referencia a un puntero que
         * sera el puntero a la muestra devuelta, o NULL si no
         * hay existe esa muestra disponible.
         */
        virtual void get_field(field_sample* &sample_ptr);
        
protected:
        /**
         * @brief Intenta detectar un robot delimitado por los puntos dados.
         */
        e_robot try_robot (cv::Point &p0, cv::Point &p1, cv::Point &p2, cv::Point &p3);

        /**
         * @brief Intenta detectar la pelota delimitada por los puntos dados.
         */
        bool try_ball (cv::Point &p0, cv::Point &p1, cv::Point &p2, cv::Point &p3);

        /**
         * @brief Intenta detectar robots y pelota en un poligono complejo.
         */
        void try_many (std::vector<cv::Point> &poly);
        
        /**
         * @brief Mantiene la muestra BGR para el color YELLOW.
         */
        double yellow_sample[3];
        /**
         * @brief Mantiene la muestra BGR para el color LIGHTBLUE.
         */
        double lblue_sample[3];
        /**
         * @brief Mantiene la muestra BGR para el color GREEN.
         */
        double green_sample[3];
        /**
         * @brief Mantiene la muestra BGR para el color VIOLET.
         */
        double violet_sample[3];

        /**
         * @brief Mantiene la muestra BGR para el color ROJO de la pelota.
         */
        double red_sample[3];

        /**
         * @brief Identifica cual de los colores YELLOW, LIGHTBLUE, GREEN o VIOLET
         * se corresponde mejor a la muestra dada.
         * Devuelve certainty un valor [0.0, 1.0) que identifica la certeza del resultado,
         * mientras más cercano a 1.0 mejor.
         */
        e_color identify_color (const cv::Scalar &sample, double &certainty);

        /**
         * @brief Identifica si la muestra dada se corresponde con el color RED.
         * Devuelve certainty un valor [0.0, 1.0) que identifica la certeza del resultado,
         * mientras más cercano a 1.0 mejor.
         */
        bool is_red (const cv::Scalar &sample, double &certainty);

        /**
         * @brief Matriz temporal para procesamientos.
         */
        cv::Mat gray0;
	cv::gpu::GpuMat d_gray0;
        /**
         * @brief Matriz temporal 2 para procesamientos.
         */
        cv::Mat gray1;
	cv::gpu::GpuMat d_gray1;

        /**
         * @brief La imagen de fondo en escala de grises.
         */
        cv::Mat background_gray;
        cv::gpu::GpuMat d_backgroundGray;
        /**
         * @brief La imagen de procesamiento actual.
         */
        cv::Mat *frame;
	//cv::gpu::GpuMat *d_frame;
        /**
         * @brief El nivel de threashold usado para distinguir los
         * colores del fondo.
         */
        double threshold;

        /**
         * @brief El nivel de umbral usado en la deteccion del color
         * rojo
         */
        double red_threashold;

        /**
         * @brief Mantiene el limite inferior para el area de un robot.
         */
        double bot_area_min;

        /**
         * @brief Mantiene el limite superior para el area de un robot.
         */
        double bot_area_max;

        /**
         * @brief Mantiene el limite inferior para el area de la pelota.
         */
        double ball_area_min;

        /**
         * @brief Mantiene el limite superior para el area de la pelota.
         */
        double ball_area_max;
        
        /**
         * @brief La cantidad de muestras a mantener.
         */
        int history_size;

        /**
         * @brief La historia de muestras mantenidas.
         */
        std::list<field_sample> field_history;

        /**
         * @brief La muestra que se esta calculando.
         */
        field_sample* current_field;

        /**
         * @brief Mantiene el tamanho del campo en pixels.
         */
        cv::Size field_size;

        /**
         * @brief Agrega el robot rid a la muestra actual.
         * Si ese robot ya existia, se elimina.
         */
        void add_history_bot  (e_robot rid, const cv::Point &p0, const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);
                
        /**
         * @brief Agrega la pelota a la muestra actual.
         */
        void add_history_ball (const cv::Point &p0, const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);
};

#endif /* BOTTRACKET_H */
