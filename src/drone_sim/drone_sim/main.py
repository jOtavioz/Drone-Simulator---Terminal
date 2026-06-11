from drone_sim.constants import *
from drone_sim.drone import Drone
from drone_sim.env import Mission, waypoints, obs1, obs2, wall
from drone_sim.controller import WaypointController
from drone_sim.physics import Physics


drone = Drone()
controller = WaypointController()
mission = Mission()
mission.addObstacle(obs1)
mission.addObstacle(obs2)
mission.addObstacle(wall)

for wp in waypoints:
    mission.addWaypoint(wp)
mission.printMission()

simulation_time = 0
MAX_TIME_PER_WAYPOINT = 60

# ============================================================
# Check waypoint reached
# ============================================================

def reachedWaypoint(drone, waypoint) -> bool:
    x_error = abs(waypoint.x - drone.position[0])
    y_error = abs(waypoint.y - drone.position[1])
    z_error = abs(waypoint.z - drone.position[2])
    # Landing: só checar Z
    if waypoint.z == 0.0:
        return z_error < 0.3
    
    return (x_error < 0.8 and y_error < 0.8 and z_error < 0.5)


# ============================================================
# Telemetry snapshot
# ============================================================

def printSnapshot(label, drone, waypoint, wp_index, sim_time):
    gps = drone.gps.read
    print(f"""
================================================================
  {label}
================================================================
  Time       : {sim_time:.1f} s
  Waypoint   : {wp_index} — {waypoint.description}
----------------------------------------------------------------
  Position        Velocity         Acceleration
  x = {drone.position[0]:>7.2f} m    vx = {drone.velocity[0]:>6.2f} m/s    ax = {drone.acceleration[0]:>6.2f} m/s²
  y = {drone.position[1]:>7.2f} m    vy = {drone.velocity[1]:>6.2f} m/s    ay = {drone.acceleration[1]:>6.2f} m/s²
  z = {drone.position[2]:>7.2f} m    vz = {drone.velocity[2]:>6.2f} m/s    az = {drone.acceleration[2]:>6.2f} m/s²
----------------------------------------------------------------
  Attitude
  Roll  = {drone.attitude[0]:>7.3f} rad    Pitch = {drone.attitude[1]:>7.3f} rad
----------------------------------------------------------------
  Motors       Power      RPM       Thrust
  FL         {drone.motors[0].power:>6.1f}%   {drone.motors[0].rpm:>6.0f}    {drone.motors[0].thrust:>6.2f} N
  FR         {drone.motors[1].power:>6.1f}%   {drone.motors[1].rpm:>6.0f}    {drone.motors[1].thrust:>6.2f} N
  RL         {drone.motors[2].power:>6.1f}%   {drone.motors[2].rpm:>6.0f}    {drone.motors[2].thrust:>6.2f} N
  RR         {drone.motors[3].power:>6.1f}%   {drone.motors[3].rpm:>6.0f}    {drone.motors[3].thrust:>6.2f} N
----------------------------------------------------------------
  GPS   x={gps[0]:>7.2f}  y={gps[1]:>7.2f}  z={gps[2]:>7.2f}
  LIDAR altitude = {drone.lidar.read:>7.2f} m
================================================================
""")


# ============================================================
# Main loop — por waypoint, 3 snapshots (início, meio, fim)
# ============================================================

for wp_index, waypoint in enumerate(mission.waypoints, start=1):

    # --- rodar física até atingir o waypoint, guardando estados ---
    steps = []                          # lista de (sim_time, cópia dos dados)
    reached = False
    waypoint_time = 0.0

    while not reached and waypoint_time < MAX_TIME_PER_WAYPOINT:

        thrust, roll_cmd, pitch_cmd, yaw_cmd = controller.calculateCommands(drone, waypoint, DT)
        drone.mixer.mix_thrust(thrust, roll_cmd, pitch_cmd, yaw_cmd)
        Physics.updatePhysics(drone, DT)
        drone.updateSensors(mission)
        simulation_time += DT
        waypoint_time += DT

        # captura snapshot como dicionário simples
        steps.append({
            "time":         simulation_time,
            "pos":          drone.position[:],
            "vel":          drone.velocity[:],
            "acc":          drone.acceleration[:],
            "att":          drone.attitude[:],
            "motors_pwr":   [m.power  for m in drone.motors],
            "motors_rpm":   [m.rpm    for m in drone.motors],
            "motors_thr":   [m.thrust for m in drone.motors],
            "gps":          drone.gps.read,
            "lidar":        drone.lidar.read,
        })

        if reachedWaypoint(drone, waypoint):
            controller.reset()
            drone.angularVelocity[0] = 0.0
            drone.angularVelocity[1] = 0.0
            reached = True

    if not steps:
        continue

    start_index = min(5, len(steps)-1)
    middle_index = len(steps)//2
    end_index = len(steps)-1

    snapshots = [
        ("INÍCIO", steps[start_index]),
        ("MEIO",   steps[middle_index]),
        ("FIM",    steps[end_index])
    ]

    # --- imprimir com pausa entre cada snapshot ---
    for label, s in snapshots:
        gps = s["gps"]
        print(f"""
================================================================
  Waypoint {wp_index} — {waypoint.description}
  [{label}]
================================================================
  Time       : {s['time']:.1f} s
----------------------------------------------------------------
  Position        Velocity         Acceleration
  x = {s['pos'][0]:>7.2f} m    vx = {s['vel'][0]:>6.2f} m/s    ax = {s['acc'][0]:>6.2f} m/s²
  y = {s['pos'][1]:>7.2f} m    vy = {s['vel'][1]:>6.2f} m/s    ay = {s['acc'][1]:>6.2f} m/s²
  z = {s['pos'][2]:>7.2f} m    vz = {s['vel'][2]:>6.2f} m/s    az = {s['acc'][2]:>6.2f} m/s²
----------------------------------------------------------------
  Attitude
  Roll  = {s['att'][0]:>7.3f} rad    Pitch = {s['att'][1]:>7.3f} rad
----------------------------------------------------------------
  Motors       Power      RPM       Thrust
  FL         {s['motors_pwr'][0]:>6.1f}%   {s['motors_rpm'][0]:>6.0f}    {s['motors_thr'][0]:>6.2f} N
  FR         {s['motors_pwr'][1]:>6.1f}%   {s['motors_rpm'][1]:>6.0f}    {s['motors_thr'][1]:>6.2f} N
  RL         {s['motors_pwr'][2]:>6.1f}%   {s['motors_rpm'][2]:>6.0f}    {s['motors_thr'][2]:>6.2f} N
  RR         {s['motors_pwr'][3]:>6.1f}%   {s['motors_rpm'][3]:>6.0f}    {s['motors_thr'][3]:>6.2f} N
----------------------------------------------------------------
  GPS   x={gps[0]:>7.2f}  y={gps[1]:>7.2f}  z={gps[2]:>7.2f}
  LIDAR altitude = {s['lidar']:>7.2f} m
================================================================
""")
        input("  [ ENTER para continuar... ]\n")

print("\n" + "=" * 70)
print("  MISSION FINISHED")
print("=" * 70)

def main():
	print("Drone Simulator")
