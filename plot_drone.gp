# GNUPlot script untuk visualisasi hasil simulasi drone
set terminal png enhanced size 1200,800
set output 'drone_analysis.png'

# Pengaturan layout multi-plot
set multiplot layout 2,2 title 'Analisis Kinerja Drone dengan Metode Runge-Kutta'

# Plot 1: Trajektori 3D
set title 'Trajektori Penerbangan Drone (3D)'
set xlabel 'X (m)'
set ylabel 'Y (m)'
set zlabel 'Z (m)'
set grid
splot 'drone_data.txt' using 2:3:4 with lines lw 2 title 'Trajektori'

# Plot 2: Ketinggian vs Waktu
set title 'Ketinggian Drone vs Waktu'
set xlabel 'Waktu (s)'
set ylabel 'Ketinggian (m)'
set grid
plot 'drone_data.txt' using 1:4 with lines lw 2 title 'Ketinggian (Z)'

# Plot 3: Sudut Euler (Roll, Pitch, Yaw)
set title 'Orientasi Drone (Sudut Euler)'
set xlabel 'Waktu (s)'
set ylabel 'Sudut (rad)'
set grid
plot 'drone_data.txt' using 1:7 with lines lw 2 title 'Roll (φ)', \
     'drone_data.txt' using 1:8 with lines lw 2 title 'Pitch (θ)', \
     'drone_data.txt' using 1:9 with lines lw 2 title 'Yaw (ψ)'

# Plot 4: Thrust dari 4 Motor
set title 'Thrust Motor vs Waktu'
set xlabel 'Waktu (s)'
set ylabel 'Thrust (N)'
set grid
plot 'drone_data.txt' using 1:13 with lines lw 2 title 'Motor 1', \
     'drone_data.txt' using 1:14 with lines lw 2 title 'Motor 2', \
     'drone_data.txt' using 1:15 with lines lw 2 title 'Motor 3', \
     'drone_data.txt' using 1:16 with lines lw 2 title 'Motor 4'

unset multiplot
