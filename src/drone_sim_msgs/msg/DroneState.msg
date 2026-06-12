float64 sim_time

# GPS — posição com ruído (m)
float64[3] gps_position

# LIDAR — altitude relativa ao obstáculo abaixo (m)
float64 lidar_altitude

# IMU (simulado) — atitude (rad) e taxas angulares (rad/s)
float64[3] attitude
float64[3] angular_velocity

# Velocidade — simplificação: estado interno do drone
float64[3] velocity
