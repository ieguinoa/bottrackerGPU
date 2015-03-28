#ifndef FIELD_SAMPLE_H
#define FIELD_SAMPLE_H

#include "ball_sample.hpp"
#include "bot_sample.hpp"
#include "utils.hpp"

/**
 * @brief Representa el estado encontrado en un frame.
 */
class field_sample {
public:
        /**
         * @brief Construye un field_sample con el timestamp dado.
         * @param timestamp El instante en el cual la informacion contenida
         * en este field_sample era valida.
         */
        field_sample(nsecs_t timestamp);
        virtual ~field_sample();
        
        /**
         * @brief Agrega un robot a la muestra.
         */
        virtual void add_robot (bot_sample sample);
        /**
         * @brief Agrega la pelota a la muestra.
         */
        virtual void add_ball (ball_sample sample);
        
        /**
         * @brief Establece sample a una muestra del robot con identificador id.
         * Retorna true si el identificador se encuentra en la muestra, o
         * false si no se encuentra, en cuyo caso sample contendra NULL.
         */
        virtual bool get_robot(e_robot id, const bot_sample* &sample) const;
        /**
         * @brief Establece sample a la muestra de la pelota.
         * Retorna true si la pelota se encuentra en la muestra, o
         * false en caso contrario, en cuyo caso sample contendra NULL.
         */
        virtual bool get_ball (const ball_sample* &sample) const;
        
        /**
         * @brief Devuelve la cantidad de robots en la muestra.
         */
        virtual int get_sampled_bots_count() const;
        /**
         * @brief Devuelve si el robot con identificador id se encuentra en la muestra.
         */
        virtual bool get_bot_sampled(e_robot rid) const;
        /**
         * @brief Devuelve si la pelota se encuentra en la muestra.
         */
        virtual bool get_ball_sampled() const;
        
        /**
         * @brief Devuelve el instante de tiempo de la muestra.
         */
        virtual nsecs_t get_timestamp() const;
        
protected:
        /**
         * @brief Mantiene el instante de la muestra.
         */
        nsecs_t timestamp;
        /**
         * @brief Mantiene la coleccion de robots sensados.
         */
        std::map<e_robot, bot_sample> robots;
        /**
         * @brief Mantiene si la pelota se encuentra en la muestra.
         */
        bool ball_set;
        /**
         * @brief Mantiene la informacion de la pelota.
         */
        ball_sample ball;
};

#endif /* FIELD_SAMPLE_H */
