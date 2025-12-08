
using DifferentialEquations
using Plots

# --- PARAMETROS DEL MODELO FISICO (Servo SG90) ---
J = 0.0001    # Inercia (kg*m^2)
B = 0.001     # Friccion viscosa
K = 0.05      # Constante electromotriz/torque
R = 2.0       # Resistencia de armadura (Ohm)
L = 0.001     # Inductancia (H)

# --- PARAMETROS DEL PID (Sintonizados) ---
Kp = 1.2
Ki = 0.5
Kd = 0.05

# Referencia (Setpoint): 0 a 2s -> 0 rad, >2s -> 1.5 rad
function reference(t)
    return (t > 2.0) ? 1.5 : 0.0
end

# --- DINAMICA DEL SISTEMA (Espacio de Estados) ---
# x[1] = Posicion (theta), x[2] = Velocidad (omega), x[3] = Corriente (i)
# x[4] = Integral del error (Estado del controlador)
function servo_dynamics!(dx, x, p, t)
    theta = x[1]
    omega = x[2]
    current = x[3]
    err_int = x[4]

    ref = reference(t)
    err = ref - theta
    
    # Ley de Control PID
    # u(t) = Kp*e + Ki*int(e) + Kd*(-omega) 
    # (Usamos derivada de salida -omega para evitar 'derivative kick')
    u = Kp * err + Ki * err_int - Kd * omega
    
    # Saturacion de voltaje (ESP32 3.3V o 5V driver)
    u = clamp(u, -5.0, 5.0)

    # Ecuaciones Diferenciales
    dx[1] = omega
    dx[2] = (K * current - B * omega) / J
    dx[3] = (u - R * current - K * omega) / L
    dx[4] = err  # Acumulacion del error integral
end

# --- EJECUCION ---
x0 = [0.0, 0.0, 0.0, 0.0]
tspan = (0.0, 4.0)
prob = ODEProblem(servo_dynamics!, x0, tspan)
sol = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)

# --- GRAFICADO ---
plot(sol, vars=(0,1), label="Posicion Real (rad)", lw=2, color=:blue)
plot!(t -> reference(t), 0, 4, label="Setpoint", ls=:dash, color=:red, lw=2)
title!("Respuesta al Escalon - Control PID Servo")
xlabel!("Tiempo (s)")
ylabel!("Posicion Angular (rad)")
