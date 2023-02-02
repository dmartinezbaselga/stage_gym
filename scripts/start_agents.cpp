#include <iostream>
#include <fstream>
#include <random>
#include <boost/algorithm/string.hpp>
#include <vector>
//#include <boost/filesystem.hpp>

using namespace std;

std::random_device rd{};
std::mt19937 generator{rd()};
static std::uniform_real_distribution<double> distributionX;
static std::uniform_real_distribution<double> distributionY;
static std::uniform_real_distribution<double> distributionTita;
static std::uniform_real_distribution<double> distributionV;
static std::uniform_real_distribution<double> distributionW;
static std::uniform_real_distribution<double> distributionFixed;

// const double xmin = -4; 
// const double xmax = 4; 
// const double ymin = -4; 
// const double ymax = 4;
double xmin = -4; 
double xmax = 4; 
double ymin = -4; 
double ymax = 4;
double vMax = 0.5, wMax = 1.0472;

const int N_COLORS = 8;
string colors[] = {"red", "green", "blue", "white", "yellow", "grey", "orange", "brown"};
int iteracion;

string targets_world = "";

class Tsc{
    public:
        double x;
        double y;
        Tsc(){}
        Tsc (double xa, double ya){
            x = xa;
            y = ya;
        }
};

double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
}

bool CollisionObs(Tsc ag1, Tsc ag2, double securityDist){

    double distancia = Distancia(ag1.x - ag2.x, ag1.y - ag2.y);

    if (distancia - securityDist < 0.0)
        return true;
    else
        return false;

}

void getVelocities (double& vx, double& vth){
    //Velocity
    std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*1.20);
    distributionV.param(parV);
    vx = distributionV(generator);

    //Angular velocity
    std::uniform_real_distribution<double>::param_type parW(-wMax*0.5, wMax*0.5);
    distributionW.param(parW);
    vth = distributionW(generator);
}

void getPosition (double& x, double& y, double& theta, vector<Tsc>& agents){
    bool done = false;

    while(!done){
        done = true;
        //x
        std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
        distributionX.param(parX);
        x = distributionX(generator);
        //y
        std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
        distributionY.param(parY);
        y = distributionY(generator);

        for (auto it=agents.begin(); it!=agents.end(); ++it){
            if (CollisionObs(*it, Tsc(x, y), (0.2)*3* 1.1)  || abs(abs(x) - 2) < 0.5 || abs(abs(y) - 2) < 0.5) {
            done = false;
            }
        }
    }
    agents.push_back(Tsc(x, y));
    //Theta
    double titamin_ini = -3.1416, titamax_ini = 3.1416;
    double titamin = titamin_ini; 
    double titamax = titamax_ini;

    if (y > ymax/3) {
        titamax =  -0.7;
    } else if (y < 0){
        titamin = 0.7;
        titamax = 1.57 + 0.7;
    }

    titamin = titamin_ini; titamax = titamax_ini;
    std::uniform_real_distribution<double>::param_type parTita(titamin, titamax);
    distributionTita.param(parTita);
    theta = distributionTita(generator);
}

void getGoal (double& xg, double& yg, const double x, const double y){
    bool done = false;
    double dist_minGoal = 4;
    while(!done){
        done = true;
        std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
        distributionX.param(parX);
        xg = distributionX(generator);
        std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
        distributionY.param(parY);
        yg = distributionY(generator);
        if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal||
                abs(abs(xg) - 2) < 0.5 || abs(abs(yg) - 2) < 0.5) {
            done = false;
        }
    }
}

bool isNumber(char c) {
    return !((c >= 48 && c <= 57) || c == '-' || c == '.');
}

void writeAMCL(ofstream& f, double x, double y, double theta, int id){
    f << "<group ns=\"robot_" << id << "\">" << endl;
    f << "<node name=\"amcl\" pkg=\"amcl\" type=\"amcl\" output=\"screen\">" << endl;

    f << "<remap from=\"scan\" to=\"ranger_0/base_scan\"/>" << endl;
    f << "<param name=\"use_map_topic\"             value=\"true\"/>" << endl;
	f << "<param name=\"odom_frame_id\" value=\"/robot_" << id << "/odom\" />" << endl;
	f << "<param name=\"base_frame_id\" value=\"/robot_" << id << "/base_link\" />" << endl;
    f << "<param name=\"global_frame_id\" value=\"/map\" />" << endl;
    f << "<remap from=\"static_map\" to=\"/static_map\"/>" << endl;
    f << "<remap from=\"map\" to=\"/map\"/>" << endl;
	f << "<!-- Initial position -->" << endl;
	f << "<param name=\"initial_pose_x\" value=\"" << x << "\" />" << endl;
    f << "<param name=\"initial_pose_y\" value=\"" << y << "\" />" << endl;
    f << "<param name=\"initial_pose_a\" value=\"" << theta << "\" />" << endl;
	
	f << "<!-- amcl specific -->" << endl;
	f << "<param name=\"odom_model_type\"           value=\"diff\"/>" << endl;
	f << "<param name=\"gui_publish_rate\"          value=\"10.0\"/>" << endl;
	f << "<param name=\"laser_max_beams\"           value=\"60\"/>" << endl;
	f << "<param name=\"laser_max_range\"           value=\"12.0\"/>" << endl;
	f << "<param name=\"min_particles\"             value=\"500\"/>" << endl;
	f << "<param name=\"max_particles\"             value=\"2000\"/>" << endl;
	f << "<param name=\"kld_err\"                   value=\"0.05\"/>" << endl;
	f << "<param name=\"kld_z\"                     value=\"0.99\"/>" << endl;
	f << "<param name=\"odom_alpha1\"               value=\"0.2\"/>" << endl;
	f << "<param name=\"odom_alpha2\"               value=\"0.2\"/>" << endl;
	f << "<!-- translation std dev, m -->" << endl;
	f << "<param name=\"odom_alpha3\"               value=\"0.2\"/>" << endl;
	f << "<param name=\"odom_alpha4\"               value=\"0.2\"/>" << endl;
    f << "<param name=\"odom_alpha5\"               value=\"0.1\"/>" << endl;
    f << "<param name=\"laser_z_hit\"               value=\"0.5\"/>" << endl;
	f << "<param name=\"laser_z_short\"             value=\"0.05\"/>" << endl;
	f << "<param name=\"laser_z_max\"               value=\"0.05\"/>" << endl;
	f << "<param name=\"laser_z_rand\"              value=\"0.5\"/>" << endl;
	f << "<param name=\"laser_sigma_hit\"           value=\"0.2\"/>" << endl;
	f << "<param name=\"laser_lambda_short\"        value=\"0.1\"/>" << endl;
	f << "<param name=\"laser_model_type\"          value=\"likelihood_field\"/>" << endl;
	f << "<param name=\"laser_likelihood_max_dist\" value=\"2.0\"/>" << endl;
	f << "<param name=\"update_min_d\"              value=\"0.1\"/>" << endl;
	f << "<param name=\"update_min_a\"              value=\"0.2\"/>" << endl; 
	f << "<param name=\"resample_interval\"         value=\"1\"/>" << endl; 
	f << "<param name=\"transform_tolerance\"       value=\"1.0\"/>" << endl;
	f << "<param name=\"recovery_alpha_slow\"       value=\"0.0001\"/>" << endl;
	f << "<param name=\"recovery_alpha_fast\"       value=\"0.1\"/>" << endl;
    f << "</node>" << endl;
    f << "</group>" << endl;
}


void insertAgent (int i, double vx, double vth, double x, double y, double theta, 
ofstream& f_world_out, ofstream& stage_out,  double xg = 0, double yg = 0, bool active = false){
    if (active){
        string color = colors[i%N_COLORS];
        f_world_out << "erratic( pose [ "; 
        f_world_out << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"robot_" << i <<"\" color \"" << color << "\")" << endl;
        writeAMCL(stage_out, x, y, theta, i);
        targets_world +=  "target( pose [ " + to_string(xg) + " " + to_string(yg) + " 0.000 0.000 ] name \"target_" +
        to_string(i) +"\" color \"" + color + "\")\n";
    }
    else{
        string color = "black";
        f_world_out << "erratic_passive( pose [ " << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"orca_" <<
        i <<"\" color \"" << color << "\")" << endl;        
    }
}

int randomScenario(int nActives, int nPasives, ofstream& stage_out, ofstream& f_world_out){
    vector<Tsc> agents_pos;
    int i;
    for (i = 0; i < nActives; i++){	
        double vx, vth, x, y, theta, xg, yg;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        getGoal(xg, yg, x, y);
        insertAgent(i, vx, vth, x, y, theta, f_world_out, stage_out, xg, yg, true);
    }
    for (; i < nPasives+nActives; i++){	
        double vx, vth, x, y, theta;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        insertAgent(i, vx, vth, x, y, theta, f_world_out, stage_out);
    }
    return nPasives + nActives;
}

void usage(int argc, char* argv[], int& nActives, int& nPasives, string& scenario)
{
    int i = 1;
    bool error = i > argc;
    // cout << "Error before while: " << i << "-" << argc << argv[argc]<< endl;
    while (!error && i < argc){
        // cout << "In while: " << string(argv[i]) << endl;
        if (string(argv[i]) == "-s" && argc > i+1){
            i++;
            scenario = argv[i];
            i++;
        }
        else if (string(argv[i]) == "-p" && argc > i+1){
            i++;
            nPasives = atoi(argv[i]);
            i++;
        }
        else if (string(argv[i]) == "-a" && argc > i+1){
            i++;
            nActives = atoi(argv[i]);
            i++;
        }
        else{
            cerr << "Error: " << string(argv[i]) << endl;
            error = true;
        }
    }
    if (error){
        cerr << "Usage: ./launch.sh [-a n_actives | -p n_pasives | -s scenario]"<< endl;
        exit(1);
    }
}

int main(int argc, char *argv[]){
    int nActives = 1, nPasives = 7;
    string scenario = "";
    usage(argc, argv, nActives, nPasives, scenario);
    ifstream f_world_in;
    if (scenario != ""){
        f_world_in.open("../world/basic_template_" + scenario + ".world");
    }
    else{
        f_world_in.open("../world/basic_template.world");
    }
    if (!f_world_in.is_open()){
        cerr << "Template file not found at ../world/basic_template.world or ../world/basic_template_" + scenario + ".world" << endl;
        return -1;
    }

    ifstream f_stage_in("../launch/stage_template.launch");

    ofstream f_stage_out("../launch/stage.launch");
    ofstream f_world_out("../world/basic.world");

    f_world_out << f_world_in.rdbuf();
    f_stage_out << f_stage_in.rdbuf();
    f_stage_out <<  "<param name=\"orca_robots\" value=\"" << nPasives << "\"/>" << endl;
    f_stage_out <<  "</node>" << endl;

    randomScenario(nActives, nPasives, f_stage_out, f_world_out);

    f_world_out << targets_world;

    if (scenario != ""){
        f_stage_out << "<node name=\"map_server\" pkg=\"map_server\" type=\"map_server\" args=\"$(find stage_ros_dovs)/world/simple_" + scenario + ".yaml\" required=\"true\"/>" << endl;
    }
    else{
        f_stage_out << "<node name=\"map_server\" pkg=\"map_server\" type=\"map_server\" args=\"$(find stage_ros_dovs)/world/simple.yaml\" required=\"true\"/>" << endl;
    }
    // f_stage_out << "<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find stage_ros_dovs)/rviz/amcl_pose.rviz\"/>" << endl;
    f_stage_out << "</launch>" << endl;

    return 0;
}