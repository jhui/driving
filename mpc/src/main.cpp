#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// With input x, compute f(x) using the trajectory model.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Create a polynomial model based on input x and y.
// order: order of the polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    // Communicate with a simulator using uWebSockets at port 4567
    uWS::Hub hub;

    // Create the Model predictive control
    MPC mpc;

    hub.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // a message always start with "42"
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                // Parse the message as a JSON message
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // For the next 6 waypoints
                    // 42["telemetry",{"ptsx":[-134.97,-145.1165,-158.3417,-164.3164,-169.3365,-175.4917],
                    //                 "ptsy":[18.404,4.339378,-17.42898,-30.18062,-42.84062,-66.52898],
                    //                 "psi_unity":3.753631,
                    //                 "psi":4.10035,
                    //                 "x":-141.0097,"y":10.43909,
                    //                 "steering_angle":-0.008731874,
                    //                 "throttle":1,
                    //                 "speed":67.09641}]

                    // j[1] is the data object
                    vector<double> ptsx = j[1]["ptsx"];  // x-position of waypoints ahead on the track (in global coordinates)
                    vector<double> ptsy = j[1]["ptsy"];  // y-position of waypoints
                    double px = j[1]["x"];               // current x-position of the vehicle (in global coordinates)
                    double py = j[1]["y"];               // current y-position
                    double psi = j[1]["psi"];            // current orientation angle of the vehicle
                    double v = j[1]["speed"];            // current vehicle speed
                    double delta = j[1]["steering_angle"];  // current vehicle steering angle
                    double a = j[1]["throttle"];            // positive for acceleration, negative for deceleration

                    // Create vectors using the linear library Eigen
                    Eigen::VectorXd ptsx_car(ptsx.size());
                    Eigen::VectorXd ptsy_car(ptsy.size());

                    // Transform the x, y waypoints from the global coordinates
                    // to the vehicle coordinates using current vehicle center and orientation
                    for (int i = 0; i < ptsx.size(); i++) {
                        double x = ptsx[i] - px;
                        double y = ptsy[i] - py;
                        ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
                        ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
                    }

                    // Fits a 3rd-order polynomial trajectory model using the waypoints
                    auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

                    // Calculate y when x = 0 using the trajectory model
                    // Save as the cte (cross-track error) in the cost equation
                    double cte = polyeval(coeffs, 0);

                    // Calculate dy/dx
                    // dy/dx = coeffs[1] since other terms contains x^k = 0 when x = 0
                    // psi = arc tan of (dy/dx)
                    // Save as epsi in the cost equation
                    double epsi = -atan(coeffs[1]);

                    // Center of gravity needed related to psi and epsi
                    const double Lf = 2.67;

                    // Assume 100 ms latency
                    const double dt = 0.1;

                    // Predict state after latency
                    // After transforming to the vehicle coordinates, x_t = y_t = psi_t = sin(psi_t0) = 0
                    // and cos(psi_t) = 1

                    double pred_px = v * dt;     // x_t+1 = x_t + v_t * cos(psi_t) * dt = v_t * dt
                    const double pred_py = 0.0;  // y_t+1 = y_t + v_t * sin(psi_t) * dt = 0

                    // psi_t+1 = psi_t + (v_t/L_f) * delta_t * dt
                    //         = (v_t/L_f) * delta_t * dt
                    double pred_psi = v * -delta / Lf * dt;

                    double pred_v = v + a * dt;  // v_t+1 = v_t + a_t * dt

                    // cte_t+1 = f(x) - y_t + v_t * sin(epsi_t) * dt
                    //         = cte + sin(epsi_t) * dt
                    double pred_cte = cte + v * sin(epsi) * dt;
                    // epsi_t+1 = (psi_t - psidest_t) + (v_t/L_f) * delta_t * dt
                    //          = epsi + (v_t/L_f) * delta_t * dt
                    double pred_epsi = epsi + v * -delta / Lf * dt;

                    // Future state
                    Eigen::VectorXd state(6);
                    state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

                    // Solve for new actuations
                    auto vars = mpc.Solve(state, coeffs);

                    // Calculate steering and throttle
                    // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
                    // Multiplying by Lf takes into account vehicle's turning ability
                    double steer_value = vars[0] / (deg2rad(25) * Lf);
                    double throttle_value = vars[1];

                    // Send values to the simulator
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    // Display the MPC predicted trajectory
                    vector<double> mpc_x_vals = {state[0]};
                    vector<double> mpc_y_vals = {state[1]};

                    // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (int i = 2; i < vars.size(); i+=2) {
                        mpc_x_vals.push_back(vars[i]);
                        mpc_y_vals.push_back(vars[i+1]);
                    }

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    // Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    double poly_inc = 2.5;
                    int num_points = 25;

                    for (int i = 1; i < num_points; i++) {
                        next_x_vals.push_back(poly_inc * i);
                        next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Introduce 100 ms Latency to mimic real driving conditions where
                    // the car doesn't actuate the commands instantly.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    hub.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    hub.onConnection([&hub](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (hub.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    hub.run();
}