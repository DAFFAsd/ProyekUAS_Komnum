#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <sstream>

/*
 * PROGRAM ANALISIS KINERJA DRONE MENGGUNAKAN METODE RUNGE-KUTTA
 * 
 * Program ini mensimulasikan dinamika penerbangan drone dengan:
 * 1. Model matematika 6-DOF (6 Degrees of Freedom)
 * 2. Metode Runge-Kutta orde 4 untuk integrasi numerik
 * 3. Visualisasi menggunakan GNU Plot
 * 
 * State Vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
 * - (x,y,z): Posisi dalam koordinat global
 * - (vx,vy,vz): Kecepatan linear dalam koordinat global
 * - (phi,theta,psi): Sudut Euler (roll, pitch, yaw)
 * - (p,q,r): Kecepatan angular dalam koordinat body
 */

class DroneSimulator {
private:
    // Parameter fisik drone (Bisa diganti sesuka hati untuk testing)
    double m = 2.0;        // Massa drone (kg)
    double g = 9.81;       // Gravitasi (m/s²)
    double Ixx = 0.029;    // Momen inersia sumbu x (kg⋅m²)
    double Iyy = 0.029;    // Momen inersia sumbu y (kg⋅m²)
    double Izz = 0.055;    // Momen inersia sumbu z (kg⋅m²)
    double l = 0.23;       // Jarak dari pusat ke motor (m)
    double b = 3.13e-5;    // Koefisien thrust (N⋅s²)
    double d = 7.5e-7;     // Koefisien drag (N⋅m⋅s²)
    double Jr = 6.5e-5;    // Inersia rotor (kg⋅m²)
    
    // Koefisien drag aerodinamis
    double Cd = 0.1;       // Koefisien drag
    double rho = 1.225;    // Densitas udara (kg/m³)
    double A = 0.05;       // Luas referensi (m²)
    
    // State vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    std::vector<double> state;
    
    // Control inputs: [u1, u2, u3, u4] (thrust dari 4 motor)
    std::vector<double> control;
    
public:
    DroneSimulator() : state(12, 0.0), control(4, 0.0) {
        // Inisialisasi posisi awal
        state[2] = 1.0; // z = 1m (ketinggian awal)
        // Contoh: Inisialisasi P (roll rate) dengan 0.1 rad/s
        state[9] = 0.1; // p (roll rate)
        state[10] = 0.2; // q (pitch rate)
        state[11] = 0.1; // r (yaw rate)
    }
    
    /*
     * FUNGSI DINAMIKA DRONE
     * Menghitung turunan state vector berdasarkan model matematika
     * 
     * Persamaan dinamika translasi:
     * m⋅a = F_thrust + F_gravity + F_drag
     * 
     * Persamaan dinamika rotasi:
     * I⋅ω̇ = τ - ω × (I⋅ω)
     */
    std::vector<double> dynamics(const std::vector<double>& x, const std::vector<double>& u) {
        std::vector<double> xdot(12, 0.0);
        
        // Ekstrak variabel state
        double vx = x[3], vy = x[4], vz = x[5];
        double phi = x[6], theta = x[7], psi = x[8];
        double p = x[9], q = x[10], r = x[11];
        
        // Matriks rotasi dari body ke global frame
        // R = Rz(psi) * Ry(theta) * Rx(phi)
        double cphi = cos(phi), sphi = sin(phi);
        double ctheta = cos(theta), stheta = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);
        
        // Elemen matriks rotasi
        double R11 = cpsi*ctheta;
        double R12 = cpsi*stheta*sphi - spsi*cphi;
        double R13 = cpsi*stheta*cphi + spsi*sphi;
        double R21 = spsi*ctheta;
        double R22 = spsi*stheta*sphi + cpsi*cphi;
        double R23 = spsi*stheta*cphi - cpsi*sphi;
        double R31 = -stheta;
        double R32 = ctheta*sphi;
        double R33 = ctheta*cphi;
        
        // Total thrust dan torsi dari control inputs
        double T = u[0] + u[1] + u[2] + u[3]; // Total thrust
        double tau_x = l * (u[1] - u[3]);     // Torsi roll
        double tau_y = l * (u[2] - u[0]);     // Torsi pitch
        double tau_z = d * (u[0] - u[1] + u[2] - u[3]); // Torsi yaw
        
        // Gaya thrust dalam koordinat body (arah -z)
        double Fx_body = 0.0;
        double Fy_body = 0.0;
        double Fz_body = -T;
        
        // Transformasi gaya ke koordinat global
        double Fx_global = R11*Fx_body + R12*Fy_body + R13*Fz_body;
        double Fy_global = R21*Fx_body + R22*Fy_body + R23*Fz_body;
        double Fz_global = R31*Fx_body + R32*Fy_body + R33*Fz_body;
        
        // Gaya drag aerodinamis
        double v_mag = sqrt(vx*vx + vy*vy + vz*vz);
        double Fx_drag = -0.5 * rho * A * Cd * v_mag * vx;
        double Fy_drag = -0.5 * rho * A * Cd * v_mag * vy;
        double Fz_drag = -0.5 * rho * A * Cd * v_mag * vz;
        
        // Persamaan kinematika posisi
        // ẋ = v
        xdot[0] = vx; // dx/dt
        xdot[1] = vy; // dy/dt
        xdot[2] = vz; // dz/dt
        
        // Persamaan dinamika translasi
        // m⋅v̇ = F_total
        xdot[3] = (Fx_global + Fx_drag) / m;         // dvx/dt
        xdot[4] = (Fy_global + Fy_drag) / m;         // dvy/dt
        xdot[5] = (-Fz_global + Fz_drag) / m - g;     // dvz/dt. -Fz_global because Fz_global as calculated is R33*(-T), so -Fz_global is R33*T (upward thrust component)
        
        // Transformasi kecepatan angular ke turunan sudut Euler
        // [φ̇]   [1  sin(φ)tan(θ)  cos(φ)tan(θ)] [p]
        // [θ̇] = [0     cos(φ)        -sin(φ)   ] [q]
        // [ψ̇]   [0  sin(φ)/cos(θ)  cos(φ)/cos(θ)] [r]
        
        xdot[6] = p + (q*sphi + r*cphi) * tan(theta); // dphi/dt
        xdot[7] = q*cphi - r*sphi;                     // dtheta/dt
        xdot[8] = (q*sphi + r*cphi) / ctheta;          // dpsi/dt
        
        // Persamaan dinamika rotasi (Euler's equation)
        // I⋅ω̇ = τ - ω × (I⋅ω)
        double Ixx_dot = (tau_x - (Izz - Iyy)*q*r) / Ixx; // dp/dt
        double Iyy_dot = (tau_y - (Ixx - Izz)*p*r) / Iyy; // dq/dt
        double Izz_dot = (tau_z - (Iyy - Ixx)*p*q) / Izz; // dr/dt
        
        xdot[9] = Ixx_dot;  // dp/dt
        xdot[10] = Iyy_dot; // dq/dt
        xdot[11] = Izz_dot; // dr/dt
        
        return xdot;
    }
    
    /*
     * METODE RUNGE-KUTTA ORDE 4
     * Algoritma integrasi numerik untuk menyelesaikan ODE
     * 
     * Formula RK4:
     * k1 = h⋅f(t_n, y_n)
     * k2 = h⋅f(t_n + h/2, y_n + k1/2)
     * k3 = h⋅f(t_n + h/2, y_n + k2/2)
     * k4 = h⋅f(t_n + h, y_n + k3)
     * 
     * y_{n+1} = y_n + (k1 + 2k2 + 2k3 + k4)/6
     */
    void rungeKutta4(double dt) {
        // k1 = h⋅f(t, x)
        std::vector<double> k1 = dynamics(state, control);
        for (auto& val : k1) val *= dt;
        
        // k2 = h⋅f(t + h/2, x + k1/2)
        std::vector<double> x_temp(12);
        for (int i = 0; i < 12; i++) {
            x_temp[i] = state[i] + k1[i] / 2.0;
        }
        std::vector<double> k2 = dynamics(x_temp, control);
        for (auto& val : k2) val *= dt;
        
        // k3 = h⋅f(t + h/2, x + k2/2)
        for (int i = 0; i < 12; i++) {
            x_temp[i] = state[i] + k2[i] / 2.0;
        }
        std::vector<double> k3 = dynamics(x_temp, control);
        for (auto& val : k3) val *= dt;
        
        // k4 = h⋅f(t + h, x + k3)
        for (int i = 0; i < 12; i++) {
            x_temp[i] = state[i] + k3[i];
        }
        std::vector<double> k4 = dynamics(x_temp, control);
        for (auto& val : k4) val *= dt;
        
        // Update state: x_{n+1} = x_n + (k1 + 2k2 + 2k3 + k4)/6
        for (int i = 0; i < 12; i++) {
            state[i] += (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) / 6.0;
        }
    }
    
    /*
     * KONTROLER PID SEDERHANA
     * Mengatur thrust motor untuk menjaga ketinggian dan stabilitas
     */
    void updateControl(double target_height = 2.0) {
        // Parameter PID untuk kontrol ketinggian
        static double integral_z = 0.0, prev_error_z = 0.0;
        double Kp_z = 0.35, Ki_z = 0.0, Kd_z = 0.55; // Reverted to PD, fine-tuning Kp and Kd
        
        // Error ketinggian
        double error_z = target_height - state[2];
        integral_z += error_z * 0.01; // dt = 0.01
        double derivative_z = (error_z - prev_error_z) / 0.01;
        prev_error_z = error_z;
        
        // Output PID untuk thrust
        double thrust_cmd = m * g + Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
        if (thrust_cmd < 0.0) {
            thrust_cmd = 0.0; // Ensure total thrust command is not negative
        }
        
        // Parameter PID untuk stabilitas attitude
        static double prev_phi = 0.0, prev_theta = 0.0;
        double Kp_att = 2.0, Kd_att = 1.0;
        
        // Kontrol roll dan pitch untuk stabilitas
        double roll_cmd = -Kp_att * state[6] - Kd_att * (state[6] - prev_phi) / 0.01;
        double pitch_cmd = -Kp_att * state[7] - Kd_att * (state[7] - prev_theta) / 0.01;
        prev_phi = state[6];
        prev_theta = state[7];
        
        // Distribusi thrust ke 4 motor
        // Thrust dasar untuk semua motor
        double base_thrust = thrust_cmd / 4.0;
        
        // Modifikasi berdasarkan roll dan pitch command
        control[0] = base_thrust - pitch_cmd / (4.0 * l); // Motor depan
        control[1] = base_thrust + roll_cmd / (4.0 * l);  // Motor kanan
        control[2] = base_thrust + pitch_cmd / (4.0 * l); // Motor belakang
        control[3] = base_thrust - roll_cmd / (4.0 * l);  // Motor kiri
        
        // Batasan thrust minimum
        for (auto& u : control) {
            if (u < 0.0) u = 0.0;
        }
    }
    
    /*
     * SIMULASI UTAMA
     * Menjalankan simulasi dengan time step tertentu
     */
    void simulate(double duration, double dt, const std::string& filename) {
        std::ofstream file(filename);
        file << "# Time X Y Z VX VY VZ Phi Theta Psi P Q R U1 U2 U3 U4\n";
        
        int steps = static_cast<int>(duration / dt);
        double t = 0.0;
        
        std::cout << "Memulai simulasi drone...\n";
        std::cout << "Durasi: " << duration << "s, Time step: " << dt << "s\n";
        std::cout << "Total steps: " << steps << "\n\n";
        
        for (int i = 0; i <= steps; i++) {
            // Update kontrol
            updateControl(2.0 + 0.5 * sin(0.5 * t)); // Target height bervariasi
            
            // Integrasi menggunakan Runge-Kutta 4
            rungeKutta4(dt);
            
            // Simpan data
            file << std::fixed << std::setprecision(6);
            file << t;
            for (const auto& val : state) {
                file << " " << val;
            }
            for (const auto& val : control) {
                file << " " << val;
            }
            file << "\n";
            
            // Progress report
            if (i % (steps/10) == 0) {
                double progress = 100.0 * i / steps;
                std::cout << "Progress: " << std::setprecision(1) << std::fixed 
                         << progress << "% - Height: " << std::setprecision(3) 
                         << state[2] << "m\n";
            }
            
            t += dt;
        }
        
        file.close();
        std::cout << "\nSimulasi selesai. Data disimpan ke: " << filename << "\n";
    }
    
    /*
     * VISUALISASI MENGGUNAKAN GNUPLOT
     * Membuat grafik trajektori dan parameter kinerja
     */
    void plotResults(const std::string& datafile) {
        std::cout << "Membuat visualisasi dengan GNUPlot...\n";
        
        // Buat script GNUPlot
        std::ofstream gnuplot_script("plot_drone.gp");
        
        gnuplot_script << "# GNUPlot script untuk visualisasi hasil simulasi drone\n";
        gnuplot_script << "set terminal png enhanced size 1200,800\n";
        gnuplot_script << "set output 'drone_analysis.png'\n\n";
        
        gnuplot_script << "# Pengaturan layout multi-plot\n";
        gnuplot_script << "set multiplot layout 2,2 title 'Analisis Kinerja Drone dengan Metode Runge-Kutta'\n\n";
        
        // Plot 1: Trajektori 3D
        gnuplot_script << "# Plot 1: Trajektori 3D\n";
        gnuplot_script << "set title 'Trajektori Penerbangan Drone (3D)'\n";
        gnuplot_script << "set xlabel 'X (m)'\n";
        gnuplot_script << "set ylabel 'Y (m)'\n";
        gnuplot_script << "set zlabel 'Z (m)'\n";
        gnuplot_script << "set grid\n";
        gnuplot_script << "splot '" << datafile << "' using 2:3:4 with lines lw 2 title 'Trajektori'\n\n";
        
        // Plot 2: Ketinggian vs Waktu
        gnuplot_script << "# Plot 2: Ketinggian vs Waktu\n";
        gnuplot_script << "set title 'Ketinggian Drone vs Waktu'\n";
        gnuplot_script << "set xlabel 'Waktu (s)'\n";
        gnuplot_script << "set ylabel 'Ketinggian (m)'\n";
        gnuplot_script << "set grid\n";
        gnuplot_script << "plot '" << datafile << "' using 1:4 with lines lw 2 title 'Ketinggian (Z)'\n\n";
        
        // Plot 3: Sudut Euler
        gnuplot_script << "# Plot 3: Sudut Euler (Roll, Pitch, Yaw)\n";
        gnuplot_script << "set title 'Orientasi Drone (Sudut Euler)'\n";
        gnuplot_script << "set xlabel 'Waktu (s)'\n";
        gnuplot_script << "set ylabel 'Sudut (rad)'\n";
        gnuplot_script << "set grid\n";
        gnuplot_script << "plot '" << datafile << "' using 1:7 with lines lw 2 title 'Roll (φ)', \\\n";
        gnuplot_script << "     '" << datafile << "' using 1:8 with lines lw 2 title 'Pitch (θ)', \\\n";
        gnuplot_script << "     '" << datafile << "' using 1:9 with lines lw 2 title 'Yaw (ψ)'\n\n";
        
        // Plot 4: Thrust Motor
        gnuplot_script << "# Plot 4: Thrust dari 4 Motor\n";
        gnuplot_script << "set title 'Thrust Motor vs Waktu'\n";
        gnuplot_script << "set xlabel 'Waktu (s)'\n";
        gnuplot_script << "set ylabel 'Thrust (N)'\n";
        gnuplot_script << "set grid\n";
        gnuplot_script << "plot '" << datafile << "' using 1:13 with lines lw 2 title 'Motor 1', \\\n";
        gnuplot_script << "     '" << datafile << "' using 1:14 with lines lw 2 title 'Motor 2', \\\n";
        gnuplot_script << "     '" << datafile << "' using 1:15 with lines lw 2 title 'Motor 3', \\\n";
        gnuplot_script << "     '" << datafile << "' using 1:16 with lines lw 2 title 'Motor 4'\n\n";
        
        gnuplot_script << "unset multiplot\n";
        gnuplot_script.close();
        
        // Jalankan GNUPlot
        system("gnuplot plot_drone.gp");
        std::cout << "Grafik berhasil dibuat: drone_analysis.png\n";
        
        // Buat analisis kinerja
        analyzePerformance(datafile);
    }
    
    /*
     * ANALISIS KINERJA DRONE
     * Menghitung metrik kinerja dari hasil simulasi
     */
    void analyzePerformance(const std::string& datafile) {
        std::ifstream file(datafile);
        std::string line;
        getline(file, line); // Skip header
        
        std::vector<double> times, heights, rolls, pitches, yaws;
        std::vector<double> vx_vals, vy_vals, vz_vals;
        
        // Baca data
        while (getline(file, line)) {
            std::istringstream iss(line);
            double t, x, y, z, vx, vy, vz, phi, theta, psi;
            if (iss >> t >> x >> y >> z >> vx >> vy >> vz >> phi >> theta >> psi) {
                times.push_back(t);
                heights.push_back(z);
                rolls.push_back(phi);
                pitches.push_back(theta);
                yaws.push_back(psi);
                vx_vals.push_back(vx);
                vy_vals.push_back(vy);
                vz_vals.push_back(vz);
            }
        }
        file.close();
        
        // Hitung metrik kinerja
        std::cout << "\n=== ANALISIS KINERJA DRONE ===\n";
        
        // Stabilitas ketinggian (RMS error)
        double target_height = 2.0;
        double height_rms = 0.0;
        for (const auto& h : heights) {
            height_rms += pow(h - target_height, 2);
        }
        height_rms = sqrt(height_rms / heights.size());
        
        // Stabilitas orientasi (RMS)
        double roll_rms = 0.0, pitch_rms = 0.0;
        for (size_t i = 0; i < rolls.size(); i++) {
            roll_rms += rolls[i] * rolls[i];
            pitch_rms += pitches[i] * pitches[i];
        }
        roll_rms = sqrt(roll_rms / rolls.size());
        pitch_rms = sqrt(pitch_rms / pitches.size());
        
        // Kecepatan maksimum
        double max_velocity = 0.0;
        for (size_t i = 0; i < vx_vals.size(); i++) {
            double v = sqrt(vx_vals[i]*vx_vals[i] + vy_vals[i]*vy_vals[i] + vz_vals[i]*vz_vals[i]);
            if (v > max_velocity) max_velocity = v;
        }
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "1. Stabilitas Ketinggian:\n";
        std::cout << "   - Target ketinggian: " << target_height << " m\n";
        std::cout << "   - RMS error ketinggian: " << height_rms << " m\n";
        std::cout << "   - Ketinggian min: " << *std::min_element(heights.begin(), heights.end()) << " m\n";
        std::cout << "   - Ketinggian max: " << *std::max_element(heights.begin(), heights.end()) << " m\n\n";
        
        std::cout << "2. Stabilitas Orientasi:\n";
        std::cout << "   - RMS Roll (φ): " << roll_rms << " rad (" << roll_rms*180/M_PI << " derajat)\n";
        std::cout << "   - RMS Pitch (θ): " << pitch_rms << " rad (" << pitch_rms*180/M_PI << " derajat)\n";
        std::cout << "   - Max Roll: " << *std::max_element(rolls.begin(), rolls.end())*180/M_PI << " derajat\n";
        std::cout << "   - Max Pitch: " << *std::max_element(pitches.begin(), pitches.end())*180/M_PI << " derajat\n\n";
        
        std::cout << "3. Kinerja Dinamik:\n";
        std::cout << "   - Kecepatan maksimum: " << max_velocity << " m/s\n";
        std::cout << "   - Durasi simulasi: " << times.back() << " s\n";
        std::cout << "   - Jumlah data points: " << times.size() << "\n\n";
        
        // Evaluasi kinerja
        std::cout << "4. Evaluasi Kinerja:\n";
        double reference_height_for_percentage = 2.0; // Using the average target height as reference
        double rms_error_percentage = (height_rms / reference_height_for_percentage) * 100.0;
        
        std::cout << "   - RMS error ketinggian (persentase dari " << reference_height_for_percentage << "m): "
                  << std::fixed << std::setprecision(2) << rms_error_percentage << "%\n";

        if (rms_error_percentage < 35.0) { // Threshold: < 35% for BAIK
            std::cout << "   ✓ Kontrol ketinggian: BAIK (RMS error < 35%)\n";
        } else if (rms_error_percentage < 50.0) { // Threshold: < 50% for SEDANG
            std::cout << "   ⚠ Kontrol ketinggian: SEDANG (RMS error < 50%)\n";
        } else {
            std::cout << "   ✗ Kontrol ketinggian: BURUK (RMS error >= 50%)\n";
        }
        
        if (roll_rms < 0.1 && pitch_rms < 0.1) {
            std::cout << "   ✓ Stabilitas orientasi: BAIK (< 6 derajat)\n";
        } else if (roll_rms < 0.2 && pitch_rms < 0.2) {
            std::cout << "   ⚠ Stabilitas orientasi: SEDANG (< 12 derajat)\n";
        } else {
            std::cout << "   ✗ Stabilitas orientasi: BURUK (> 12 derajat)\n";
        }
        
        std::cout << "\n=== SELESAI ===\n";
    }
};

/*
 * FUNGSI UTAMA
 * Entry point program
 */
int main() {
    std::cout << "===============================================\n";
    std::cout << "    SIMULATOR ANALISIS KINERJA DRONE\n";
    std::cout << "    Menggunakan Metode Runge-Kutta Orde 4\n";
    std::cout << "===============================================\n\n";
    
    // Inisialisasi simulator
    DroneSimulator drone;
    
    // Parameter simulasi
    double duration = 20.0;  // 20 detik
    double dt = 0.01;        // Time step 10ms
    std::string datafile = "drone_data.txt";
    
    std::cout << "Parameter Simulasi:\n";
    std::cout << "- Durasi: " << duration << " detik\n";
    std::cout << "- Time step: " << dt << " detik\n";
    std::cout << "- Metode integrasi: Runge-Kutta Orde 4\n";
    std::cout << "- Model: 6-DOF (6 Degrees of Freedom)\n\n";
    
    // Jalankan simulasi
    drone.simulate(duration, dt, datafile);
    
    // Buat visualisasi
    drone.plotResults(datafile);
    
    std::cout << "\nUntuk melihat hasil:\n";
    std::cout << "1. Buka file 'drone_analysis.png' untuk melihat grafik\n";
    std::cout << "2. Buka file 'drone_data.txt' untuk melihat data numerik\n";
    std::cout << "3. Script GNUPlot tersimpan di 'plot_drone.gp'\n";
    
    return 0;
}