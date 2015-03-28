#ifndef TYPES_H
#define TYPES_H

/**
 * @brief Enumerativo para los colores.
 */
typedef enum {
        YELLOW = 0,
        LIGHTBLUE ,
        VIOLET    ,
        GREEN     ,
        RED       ,
        COLOR_UNKNOWN
} e_color;

/**
 * @brief Enumerativo de los robots.
 * Los robots LB_x son los del equipo celeste.
 * Los robots YY_x son los del equipo amarillos.
 */
typedef enum {
        LB_0 = 0, LB_1, LB_2, LB_3, LB_4,
        YY_0 = 5, YY_1, YY_2, YY_3, YY_4,
        ROBOT_UNKNOWN
} e_robot;

#endif /* TYPES_H */
