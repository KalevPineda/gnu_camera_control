
using DifferentialEquations
using Plots

# --- 1. DEFINICIÓN DE PARÁMETROS (Tupla con nombre) ---
# Empaquetamos todo en 'p' para evitar variables globales
p_sys = (
    J  = 0.0001,   # Inercia (kg*m^2)
    B  = 0.001,    # Friccion viscosa
    K  = 0.05,     # Constante electromotriz/torque
    R  = 2.0,      # Resistencia de armadura (Ohm)
    L  = 0.001,    # Inductancia (H)
    Kp = 1.2,      # Ganancia Proporcional
    Ki = 0.5,      # Ganancia Integral
    Kd = 0.05      # Ganancia Derivativa
)

# Función de referencia (Setpoint)
function reference(t)
    return (t > 2.0) ? 1.5 : 0.0
end

# --- 2. DINÁMICA DEL SISTEMA ---
function servo_dynamics!(dx, x, p, t)
    # Desempaquetamos los parámetros desde 'p'
    J, B, K, R, L, Kp, Ki, Kd = p.J, p.B, p.K, p.R, p.L, p.Kp, p.Ki, p.Kd
    
    # Estados actuales
    theta   = x[1] # Posición
    omega   = x[2] # Velocidad
    current = x[3] # Corriente
    # err_int = x[4] # No necesitamos leerlo para calcular dx, solo para el PID si fuera necesario
    err_int = x[4] 

    # Error
    ref = reference(t)
    err = ref - theta
    
    # Ley de Control PID
    # u(t) = Kp*e + Ki*int(e) - Kd*omega (Derivada en la medición)
    u = Kp * err + Ki * err_int - Kd * omega
    
    # Saturación de voltaje
    u = clamp(u, -5.0, 5.0)

    # Ecuaciones Diferenciales
    dx[1] = omega
    dx[2] = (K * current - B * omega) / J  # Aceleración angular
    dx[3] = (u - R * current - K * omega) / L # Cambio en corriente
    dx[4] = err  # Acumulación del error (Integral)
end

# --- 3. EJECUCIÓN ---
x0 = [0.0, 0.0, 0.0, 0.0] # [theta, omega, current, int_error]
tspan = (0.0, 4.0)

# Pasamos 'p_sys' como el cuarto argumento al ODEProblem
prob = ODEProblem(servo_dynamics!, x0, tspan, p_sys)
sol = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)

# --- 4. GRAFICADO ---
# vars=(0,1) significa Eje X: Tiempo (índice 0), Eje Y: Estado 1 (Posición)
dinamic = plot(sol, idxs=(0,1), label="Posición Real (rad)", lw=2, color=:blue, legend=:bottomright)

# Graficamos la referencia sobre la misma figura
plot!(t -> reference(t), 0, 4, label="Setpoint", ls=:dash, color=:red, lw=2)
savefig(dinamic, "example.png")

title!("Respuesta al Escalón - Control PID Servo")
xlabel!("Tiempo (s)")
ylabel!("Posición Angular (rad)")
