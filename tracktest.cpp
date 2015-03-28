#include "bottracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <vector>
#include <iostream>
#include <sstream>

using std::stringstream;
using std::vector;
using std::string;

using std::cout;
using std::endl;
using namespace cv;

CvFont font;

#define SHOW_PERIMETERS 1
#define SHOW_STATS      2

int show_flags = SHOW_PERIMETERS | SHOW_STATS;

vector<nsecs_t> processing_times;

string l_stats("");

void show_robot_sample(cv::Mat &output, const bot_sample* bs)
{
        const vector<cv::Point> pts = bs->get_shape();
        cv::Point c = bs->get_center();
        std::string bname;

        if (show_flags & SHOW_PERIMETERS) {
                cv::Point p0 = pts[0];
                for (int i = 0; i != (int)pts.size(); ++i) {
                        cv::Point p1 = pts[i];
                        cv::Point p2 = pts[(i+1)%pts.size()];
                        cv::line(output, p1, p2, cv::Scalar(0,255,0));
                }
                cv::rectangle(output, p0, p0+cv::Point(2,2), cv::Scalar::all(255));

                get_robot_name(bs->get_id(), bname);
                //cv::addText(output, bname, c-cv::Point(5,-3), font);
                cv::Point orient = bs->get_orientation();

                cv::line(output, c, c+orient, cv::Scalar(255,255,255), 2);
        }

        if (show_flags & SHOW_STATS) {
                stringstream sc, ss, so, sa;
                sc << "c: " << c;
                //cv::addText(output, sc.str(), c + cv::Point(20, -10), font);
                ss << "s: " << bs->get_speed();
                //cv::addText(output, ss.str(), c + cv::Point(20, 0), font);
                so << "o: " << bs->get_orientation();
                //cv::addText(output, so.str(), c + cv::Point(20, 10), font);
                sa << "a: " << bs->get_area();
                //cv::addText(output, sa.str(), c + cv::Point(20, 20), font);
        }
}

void show_ball_sample(cv::Mat &output, const ball_sample* bs)
{
        if (show_flags & SHOW_PERIMETERS) {
                const vector<cv::Point> pts = bs->get_shape();
                cv::circle(output, bs->get_center(), 8, cv::Scalar(0,0,255));
                //cv::addText(output, "B", bs->get_center(), font);
        }
        if (show_flags & SHOW_STATS) {
                stringstream sc, ss, sa;
                cv::Point c = bs->get_center();
                sc << "c" << c;
                //cv::addText(output, sc.str(), c + cv::Point(20, -5), font);
                ss << "s" << bs->get_speed();
                //cv::addText(output, ss.str(), c + cv::Point(20, 5), font);
                sa << "a: " << bs->get_area();
                //cv::addText(output, sa.str(), c + cv::Point(20, 15), font);
        }
}

void show_field_sample(cv::Mat &output, const field_sample* fs)
{
        if (fs != NULL) {
                int rid;
                const bot_sample* bot;
                const ball_sample * ball;
                //std::cout  << "ROBOT_UNKNOWN = "  << ROBOT_UNKNOWN;
		//for (rid = 0; rid < (int)ROBOT_UNKNOWN; ++rid) {
                  //      if (fs->get_robot((e_robot)rid, bot))
                                //show_robot_sample(output, bot);
                //}
                //if (fs->get_ball(ball))
                        //show_ball_sample(output, ball);
        }
}

void show_stats(cv::Mat &output)
{
        if ((show_flags & SHOW_STATS) && processing_times.size() > 1) {
                double mean = 0.0;
                double min  = std::numeric_limits<nsecs_t>::max();
                double max  = 0.0;
                double curr = processing_times.back();
                double dev_squared = 0.0;
                size_t values = 0;
                for (size_t i = 0; i < processing_times.size(); ++i) {
                        double v = processing_times[i];
                        mean += v;
                        if (v < min) min = v;
                        if (v > max) max = v;
                        ++values;
                }
                mean /= values;
                for (size_t i = 0; i < processing_times.size(); ++i) {
                        double deviation = processing_times[i]-mean;
                        dev_squared += deviation*deviation;
                }
                // Desviacion estandard de una muestra.
                double stddev = sqrt(dev_squared / (values - 1));
                
                std::stringstream stats;
                stats
                        << "frames: " << processing_times.size()
                        << " - proc. time: " << N2USECS(curr) << " us."
                        << " - mean: "     << N2USECS(mean) << " us."
                        << " - std. dev.: " << N2USECS(stddev) << " us."
                        << " - min: " << N2USECS(min) << " us."
                        << " - max: " << N2USECS(max) << " us.";
                l_stats = stats.str();
                //cv::addText(output, l_stats, cv::Point(10,460), font);
        }
}

string prog_name = "";
string avi_file  = "";
string bmp_file  = "";

void usage()
{
        cout <<
"usage: " << prog_name << " videofile [background]\n"
"where videofile is a robots camera file in avi format "
"and background is a .bmp file with the "
"background of the video.\n"
"If background is not given, "<< prog_name << " will "
"try using the same videofile with .bmp extension.\n\n"
"Keys:\n"
" [SPACE] : toggle pause\n"
" [ESC]   : exit\n"
" s       : toggle statistics\n"
" p       : toggle perimeters\n"
" +       : increase speed\n"
" -       : decrease speed\n"
" .       : step by step, continue with [SPACE]\n"
        << endl;
}

bool read_params(int argc, const char ** argv)
{
        bool retval = true;
        prog_name = string(argv[0]);
        size_t avilen(0);
        if (argc > 1) {
                avi_file = argv[1];
                avilen = avi_file.size();
                if (avilen < 4 || avi_file.substr(avilen-4, 4) != ".avi")
                        retval = false;
        }
        if (argc > 2) {
                bmp_file = argv[2];
        } else {
                if (retval) {
                        bmp_file = avi_file.substr(0, avi_file.size()-4) + ".bmp";
                }
        }
        if (argc > 3)
                retval = false;
        return avilen >= 4 && retval;
}

int main (int argc, char** argv)
{
/*
../tests/20110601/test1.avi
../tests/20110601/test2.avi
../tests/20110601/test3.avi
../tests/20110601/test4.avi
*/

        if (!read_params(argc, (const char**)argv)) {
                usage();
                return EXIT_FAILURE;
        }
        cout << "Video     : " << avi_file << endl;
        cout << "Background: " << bmp_file << endl;
        //font = fontQt("Courier", 8, cv::Scalar(255, 255, 255));
        
        cv::Mat background = cv::imread(bmp_file.c_str());
        cv::Mat frame;
        
	if (background.data == NULL) {
                std::cerr << "Invalid background: " << bmp_file << std::endl;
                usage();
                return EXIT_FAILURE;
        }

        //cv::namedWindow("w0", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

        bool finished = false;

        cv::VideoCapture capture(avi_file.c_str());
        double fps = capture.get(CV_CAP_PROP_FPS);
        double frame_time = 1000.0 / fps; // tiempo en ms de 1 frame.

        bot_tracker bt(background, 1);
        bt.set_color_code(YELLOW   , 0.33, 0.82, 1.00);
        bt.set_color_code(LIGHTBLUE, 1.00, 0.82, 0.61);
        bt.set_color_code(VIOLET   , 0.91, 0.66, 1.00);
        bt.set_color_code(GREEN    , 0.55, 1.00, 0.75);
        bt.set_color_code(RED      , 0.32, 0.43, 1.00);
        bt.set_threshold(25);
        bt.set_redcolor_threshold(0.65);
        bt.set_bot_area_limits(550, 1300);
        bt.set_ball_area_limits(100, 200);

        int frame_cnt = 0;
        int key;
        bool paused = false;
        
        nsecs_t ptime;
        while (!finished) {
                capture >> frame;
		
	        if (frame.data) {
                        //cout << "Procesando el frame:" << (frame_cnt+1) << endl;
			ptime = bt.process(frame);
                        processing_times.push_back(ptime);
			
                        //cout << "El tiempo para procesar este frame es:" << ptime << "nanosec." << endl; 
			field_sample* fs = NULL;
                        bt.get_field(fs);
                        
                        show_field_sample(frame, fs);
                        
			//ESTE METODO NOOO USA EL FRAME, SOLO PROCESA LAS ESTATISTICAS
			show_stats(frame);
                        
                        //cv::imshow("w0", frame);
                        if (paused)
                                key = cv::waitKey(0);
                        else {
                                double sleeptime = frame_time - N2MSECS(ptime);
                                if (sleeptime <= 0) {
                                        cout << "[WARN] : processing time too long, system will not operate in real time!" << endl;
                                        sleeptime = 0;
                                }
                                key = cv::waitKey(sleeptime);
                        }

                        switch (key) {
                                // ESC key
                                case 27: finished = true; break;
                                case ' ': paused = !paused; break;
                                case 'p': show_flags ^= SHOW_PERIMETERS; break;
                                case 's': show_flags ^= SHOW_STATS; break;
                                case '.': paused = true; break;
                                case '+': frame_time /= 2; break;
                                case '-': frame_time *= 2; break;
                                default:
                                        if (key != -1)
                                                cout << "Unknown key: " << (int)key << endl;
                                        break;
                        }
                        if (frame_time < 1)
                                frame_time = 1;
                } else {
                        finished = true;
                }
                ++frame_cnt;
        }
	std::cout  << "ROBOT_UNKNOWN = "  << ROBOT_UNKNOWN;

        capture.release();
        //cv::destroyWindow("w0");
        cout << l_stats << endl;

        return EXIT_SUCCESS;
}

