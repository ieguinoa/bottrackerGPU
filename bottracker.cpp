#include "bottracker.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "utils.hpp"
#include <sys/time.h>


using cv::Mat;
using cv::Scalar;
using cv::Point;

using std::map;
using std::make_pair;
using std::vector;
using std::list;
using std::string;



//********************
// Utilidades

/**
 * @brief Rota los cuatro colores de un robot y las posiciones donde
 * se encontraron dichos colores.
 */
void rotate(
        e_color &a , e_color &b , e_color &c , e_color &d,
        Point   &ap, Point   &bp, Point   &cp, Point &dp
) {
        e_color tmp;
        Point tmpp;
        tmp = a; a = b; b = c; c = d; d = tmp;
        tmpp=ap; ap=bp; bp=cp; cp=dp; dp=tmpp;
}

/**
 * @brief Identifica el robot del equipo key.
 * Hay 5 patrones de robot por equipo, con key el color del equipo
 * (amarillo o celeste).
 * Los patrones son:
 * robot 0: KEY, KEY, GREEN, GREEN,
 * robot 1: KEY, KEY, VIOLET, VIOLET.
 * robot 2: KEY, KEY, GREEN, VIOLET.
 * robot 3: KEY, KEY, VIOLET, GREEN.
 * robot 4: KEY, VIOLET, KEY, GREEN.
 */
e_robot robot_id (
        e_color key,
        e_color &a, e_color &b, e_color &c, e_color &d,
        cv::Point &ap, cv::Point &bp, cv::Point &cp, cv::Point &dp
) {
        e_robot id = key == YELLOW ? YY_0 : LB_0;
        if ( (a == key && c == key) || (b == key && d == key)) {
                if (a != key)
                        rotate(a,b,c,d,ap,bp,cp,dp);
                if (b != VIOLET) {
                        rotate(a,b,c,d,ap,bp,cp,dp);
                        rotate(a,b,c,d,ap,bp,cp,dp);
                }

                if (b != VIOLET || d != GREEN)
                        return ROBOT_UNKNOWN;
                return (e_robot)(id+4);
        }
        int rotations = 0;
        while (a != key || b != key) {
                rotate(a,b,c,d,ap,bp,cp,dp);
                if ( (++rotations) > 3) {
                        return ROBOT_UNKNOWN;
                }
        }
        if (c == GREEN  && d ==  GREEN)
                return (e_robot)(id);
        if (c == VIOLET && d == VIOLET)
                return (e_robot)(id+1);
        if (c == GREEN  && d == VIOLET)
                return (e_robot)(id+2);
        if (c == VIOLET && d ==  GREEN)
                return (e_robot)(id+3);
        return ROBOT_UNKNOWN;
}

/**
 * @brief Identifica un robot.
 * Primero intenta determinar de que equipo es el robot
 * y despues identifica cual de los 5 del equipo es.
 */
e_robot robot_id (
        e_color &a, e_color &b, e_color &c, e_color &d,
        cv::Point &ap, cv::Point &bp, cv::Point &cp, cv::Point &dp
) {
        e_robot rid = ROBOT_UNKNOWN;
        if (a == YELLOW || b == YELLOW || c == YELLOW || d == YELLOW) {
                rid = robot_id(YELLOW, a, b, c, d, ap, bp, cp, dp);
        } else {
                rid = robot_id(LIGHTBLUE, a, b, c, d, ap, bp, cp, dp);
        }
        if (a == YELLOW && b == a && c == a && d == a && rid != ROBOT_UNKNOWN) {
                std::cout << "[WARN] Too many yellows!" << std::endl;
        }
        return rid;
}

bot_tracker::bot_tracker(cv::Mat &background, int history_size)
: threshold(25)
, red_threashold(0.65)
, bot_area_min(600)
, bot_area_max(1200)
, ball_area_min(100)
, ball_area_max(200)
, history_size(history_size)
, field_size(cv::Size(0, 0))
{
        // Valores teoricos: no son muy utiles.
        set_color_code(YELLOW   , 0.33, 0.96, 1.00);
        set_color_code(LIGHTBLUE, 1.00, 0.80, 0.33);
        set_color_code(VIOLET   , 1.00, 0.33, 0.80);
        set_color_code(GREEN    , 0.33, 1.00, 0.41);

        // me quedo con la version en escala de grises del fondo.
        //cv::cvtColor(background, background_gray, CV_BGR2GRAY);
	
	//pasar el background a escala de grises en gpu
        d_backgroundGray.upload(background_gray);  
	
	

}

bot_tracker::~bot_tracker()
{
}

void bot_tracker::set_color_code(e_color color_id, double blue, double green, double red)
{
        normalize_color(blue, green, red);
        double *sample = NULL;
        switch (color_id) {
                case YELLOW   : sample = yellow_sample; break;
                case LIGHTBLUE: sample = lblue_sample ; break;
                case GREEN    : sample = green_sample ; break;
                case VIOLET   : sample = violet_sample; break;
                case RED      : sample = red_sample   ; break;
                default: break;
        }
        if (sample != NULL) {
                sample[0] = blue;
                sample[1] = green;
                sample[2] = red;
        }
}

void bot_tracker::set_threshold(double threshold)
{
        this->threshold = threshold;
}

void bot_tracker::set_redcolor_threshold(double threshold)
{
        this->red_threashold = threshold;
}

void bot_tracker::set_bot_area_limits(double min, double max)
{
        bot_area_min = min;
        bot_area_max = max;
}

void bot_tracker::set_ball_area_limits(double min, double max)
{
        ball_area_min = min;
        ball_area_max = max;
}

e_color bot_tracker::identify_color(const cv::Scalar& sample, double& certainty)
{
        double c0 = sample[0];
        double c1 = sample[1];
        double c2 = sample[2];
        normalize_color(c0, c1, c2);

        double y = pow(c0-yellow_sample[0], 2)+pow(c1-yellow_sample[1], 2)+pow(c2-yellow_sample[2], 2);
        double l = pow(c0-lblue_sample[0], 2)+pow(c1-lblue_sample[1], 2)+pow(c2-lblue_sample[2], 2);
        double v = pow(c0-violet_sample[0], 2)+pow(c1-violet_sample[1], 2)+pow(c2-violet_sample[2], 2);
        double g = pow(c0-green_sample[0], 2)+pow(c1-green_sample[1], 2)+pow(c2-green_sample[2], 2);
        
        if (y<l && y<v && y<g) { certainty = 1.0-sqrt(y); return YELLOW   ; }
        if (l<y && l<v && l<g) { certainty = 1.0-sqrt(l); return LIGHTBLUE; }
        if (v<y && v<l && v<g) { certainty = 1.0-sqrt(v); return VIOLET   ; }
        if (g<y && g<l && g<v) { certainty = 1.0-sqrt(g); return GREEN    ; }
        certainty = 0.0;
        return COLOR_UNKNOWN;
}

bool bot_tracker::is_red (const Scalar &sample, double &certainty)
{
        certainty = 0.0;
        double c0 = sample[0];
        double c1 = sample[1];
        double c2 = sample[2];
        normalize_color(c0, c1, c2);
        
        certainty = 1.0 - sqrt(pow(c0-red_sample[0], 2)+pow(c1-red_sample[1], 2)+pow(c2-red_sample[2], 2));

        return certainty > red_threashold;
}

void bot_tracker::try_many(vector<Point> &points)
{
        int npoints = (int)points.size();
        if (npoints < 3)
                return;
        
        int fst;
        for (fst = 0; fst < npoints; ++fst) {
                Point p0 = points[fst];
                Point p1 = points[(fst+1) % npoints];
                Point p2 = points[(fst+2) % npoints];
                Point p3 = p0+(p2-p1); // proyectado.
                if (in_bounds(p3, field_size) && is_left(p0, p1, p2)) {
                        bool tryball = !current_field->get_ball_sampled();
                        e_robot rid = try_robot(p0, p1, p2, p3);
                        if (rid != ROBOT_UNKNOWN && !current_field->get_bot_sampled(rid)) {
                                add_history_bot(rid, p0, p1, p2, p3);
                                tryball = false;
                        }
                        if (tryball && try_ball(p0, p1, p2, p3)) {
                                add_history_ball(p0, p1, p2, p3);
                        }
                }
        }
}

e_robot bot_tracker::try_robot (Point &p0, Point &p1, Point &p2, Point &p3)
{
        e_robot rid = ROBOT_UNKNOWN;
        double area = polygon_area(p0, p1, p2, p3);

        if (area >= bot_area_min && area <= bot_area_max) {
                // vectores diagonales del cuadrado.
                Point d1vect = p2-p0;
                Point d2vect = p3-p1;

                // centros de los colores: a 1/4 y 3/4 de cada diagonal.
                Point d1 = p0+.25*d1vect-cv::Point(2,2);
                Point d2 = p1+.25*d2vect-cv::Point(2,2);
                Point d3 = p0+.75*d1vect-cv::Point(2,2);
                Point d4 = p1+.75*d2vect-cv::Point(2,2);

                // busco el color promedio en cada punto, usando
                // una region de 5x5 pixeles.
                Mat roi_0(*frame, cv::Rect(d1, d1+cv::Point(5, 5)));
                Mat roi_2(*frame, cv::Rect(d3, d3+cv::Point(5, 5)));
                Mat roi_1(*frame, cv::Rect(d2, d2+cv::Point(5, 5)));
                Mat roi_3(*frame, cv::Rect(d4, d4+cv::Point(5, 5)));
          
		Scalar c0 = cv::mean(roi_0);
                Scalar c2 = cv::mean(roi_2);
                Scalar c1 = cv::mean(roi_1);
                Scalar c3 = cv::mean(roi_3);

                // identifico los colores detectados contra la muestra.
                // TODO: certainty no se usa, sirve?
                double certainty = 0.0;
                e_color a = identify_color(c0, certainty);
                e_color b = identify_color(c1, certainty);
                e_color c = identify_color(c2, certainty);
                e_color d = identify_color(c3, certainty);
                
                rid = robot_id(a,b,c,d,p0,p1,p2,p3);
        }

        return rid;
}

bool bot_tracker::try_ball (Point &p0, Point &p1, Point &p2, Point &p3)
{
        bool found = false;
        double area = polygon_area(p0, p1, p2, p3);

        // TODO: quitar magic number
        if (area >= ball_area_min && area <= ball_area_max) {
                // centro de la pelota.
                Point center = (p2-p0)*.5 + p0 - Point(1,1);
                // me quedo con el color promedio en un area de 3x3
                Mat roi(*frame, cv::Rect(center, center+cv::Point(3, 3)));
                Scalar color = cv::mean(roi);
                double certainty;
                found = is_red(color, certainty);
        }
        return found;
}

void bot_tracker::add_history_bot  (
        e_robot rid,
        const Point& p0,
        const Point& p1,
        const Point& p2,
        const Point& p3)
{
        vector<Point> vp;
        pts2vec(vp, p0, p1, p2, p3);
        bot_sample bs(rid);
        Point center = p0+0.5*(p2-p0);
        Point orientation = p0+0.5*(p1-p0)-center;
        Point speed(0,0);
        if (this->field_history.size() != 0) {
                field_sample &latest = field_history.front();
                const bot_sample* last_bot;
                if (latest.get_robot(rid, last_bot)) {
                        speed = center-last_bot->get_center();
                }
        }
        bs.set_center(center);
        bs.set_orientation(orientation);
        bs.set_speed(speed);
        bs.set_shape(vp);
        current_field->add_robot(bs);
}

void bot_tracker::add_history_ball (
        const Point& p0,
        const Point& p1,
        const Point& p2,
        const Point& p3)
{
        vector<Point> vp;
        pts2vec(vp, p0, p1, p2, p3);
        ball_sample bs;
        Point center = p0+0.5*(p2-p0);
        Point speed(0,0);
        if (this->field_history.size() != 0) {
                field_sample &latest = field_history.front();
                const ball_sample* last_ball;
                if (latest.get_ball(last_ball)) {
                        speed = center-last_ball->get_center();
                }
        }
        bs.set_center(center);
        bs.set_speed(speed);
        bs.set_shape(vp);
        current_field->add_ball(bs);
}

nsecs_t bot_tracker::process(Mat& f, cv::gpu::GpuMat& d_frame)
{

 
        field_sample field(get_timestamp());
        this->current_field = &field;
        this->frame = &f;
        this->field_size = frame->size();
       

	// Vector donde guardo los blobs no identificados
        // en la primera pasada.
        vector< vector<Point> > ublobs;

        // Vector de contornos de potenciales robots encontrados.
        vector< vector<Point> > contours;
        vector< vector<Point> >::iterator contour_it;
        vector< vector<Point> >::iterator ublob_it;
	

	//subo el frame a GPU
	d_frame.upload(f);

	//conversion a escala de grises del frame
	cv::gpu::cvtColor(d_frame,d_gray0,CV_BGR2GRAY);

	//diferencia absoluta
	cv::gpu::absdiff(d_gray0,d_backgroundGray,d_gray1);
	
	
	//aplico threshold
	cv::gpu::threshold(d_gray1,d_gray1,threshold, 255, CV_THRESH_BINARY);
	
	
	//descargo el resultado a memoria de CPU y continuo trabajando sobre gray1
	d_gray1.download(gray1);
	

	//*************A PARTIR DE ACA LA GPU NO INTERVIENE MAS ***************
	//**********************************************************************	

	// busca los contornos de los potenciales robots.
        cv::findContours(gray1, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	

	//std::cout  << "Numero de contornos encontrados = "  << contours.end();
	int algoEncontrado=0;
        // Itera los contornos encontrados para identificar robots y pelota.
        for (contour_it = contours.begin(); contour_it != contours.end(); ++contour_it)
        {
                vector<Point> polygon;
                bool robot_found = false;
                bool ball_found  = false;
		algoEncontrado++;
                // TODO: 5 magic number?
                cv::approxPolyDP(cv::Mat(*contour_it), polygon, 5, true);

                if (polygon.size() == 4 && cv::isContourConvex(cv::Mat(polygon))) {
                        Point p0, p1, p2, p3;
                        vec2pts(polygon, p0, p1, p2, p3);
                        
                        e_robot rid = try_robot(p0, p1, p2, p3);
                        if (rid != ROBOT_UNKNOWN) {
                                add_history_bot(rid, p0, p1, p2, p3);
                                robot_found = true;
                        }
                        if (!robot_found) {
                                ball_found = try_ball(p0, p1, p2, p3);
                                if (ball_found)
                                        add_history_ball(p0, p1, p2, p3);
                        }
                }
                if (!robot_found && !ball_found)
                        ublobs.push_back(polygon);
        }
	//std::cout  << "Numero de cosas encontradass = "  << algoEncontrado;
       // Itera los no encontrados con un algoritmo mas pesado.
        for (ublob_it = ublobs.begin(); ublob_it != ublobs.end(); ++ublob_it)
        {
                vector<Point> &vp = *ublob_it;
                double area = fabs(cv::contourArea(cv::Mat(vp)));
                if (area > ball_area_min) {
                        try_many(vp);
                }
        }
        field_history.push_front(field);
        if ((int)field_history.size() > history_size)
                field_history.pop_back();
        //std::cout <<  "Robots en el campo = " << current_field->get_sampled_bots_count() << std::endl; 
	return get_timestamp() - field.get_timestamp();
}

void bot_tracker::get_field(field_sample* &sample_ptr)
{
        if (field_history.empty())
                sample_ptr = NULL;
        else
                sample_ptr = &field_history.front();
        
}
