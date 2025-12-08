

# Pkg.add(["Plots", "DifferentialEquations", "LaTeXStrings"])

using Plots
using DifferentialEquations
using LaTeXStrings

# --- MODELO DE LA PLANTA (MOTOR DC SIN CONTROL INTERNO) ---
# Ecuación: J*d²θ/dt² + b*dθ/dt = Torque(u)
const J = 0.02    # Inercia del rotor
const b = 0.15    # Fricción viscosa (amortiguamiento natural bajo)
const K_m = 5.0   # Constante del motor (Voltaje -> Torque)

# Estado: u[1] = posición, u[2] = velocidad
# Parámetro p[1] = voltaje aplicado (salida del PID)
function dc_motor_physics!(du, u, p, t)
    theta, omega = u
    voltage = p[1] # Señal de control u(t)
    
    du[1] = omega
    # Aceleración = (Torque - Fricción) / Inercia
    du[2] = (K_m * voltage - b * omega) / J
end

# --- ESTRUCTURA PID DIGITAL ---
mutable struct PIDController
    Kp::Float64
    Ki::Float64
    Kd::Float64
    prev_error::Float64
    integral::Float64
    output_limits::Tuple{Float64, Float64}
    sample_time::Float64
    last_compute_time::Float64
end

function compute_pid(pid::PIDController, setpoint, measured_val, t)
    dt = t - pid.last_compute_time
    if dt < pid.sample_time
        return nothing # No es momento de calcular aún
    end
    
    # 1. Calcular Error
    error = setpoint - measured_val
    
    # 2. Términos
    P = pid.Kp * error
    
    pid.integral += error * dt
    I = pid.Ki * pid.integral
    
    derivative = (error - pid.prev_error) / dt
    D = pid.Kd * derivative
    
    # 3. Salida cruda
    output = P + I + D
    
    # 4. Anti-Windup y Saturación
    min_out, max_out = pid.output_limits
    if output > max_out
        output = max_out
        pid.integral -= error * dt # Clamping (evitar windup)
    elseif output < min_out
        output = min_out
        pid.integral -= error * dt
    end
    
    # Actualizar estado
    pid.prev_error = error
    pid.last_compute_time = t
    
    return output
end

function run_pid_simulation()
    # --- CONFIGURACIÓN ---
    t_end = 8.0
    dt_sim = 0.001 # Resolución física fina
    
    # Sintonización del PID (Tuning)
    # Prueba cambiar estos valores para ver inestabilidad!
    my_pid = PIDController(
        1.2,   # Kp: Fuerza proporcional (resorte virtual)
        0.5,   # Ki: Elimina el error estacionario
        0.08,  # Kd: Frena las oscilaciones (amortiguador)
        0.0,   # prev_error
        0.0,   # integral
        (-12.0, 12.0), # Límites de voltaje (+-12V)
        0.02,  # Tiempo de muestreo (20ms = 50Hz)
        0.0    # last time
    )
    
    # Trayectoria de Referencia (Multi-Step)
    function get_setpoint(t)
        if t < 2.0 return 45.0
        elseif t < 4.0 return 90.0
        elseif t < 6.0 return 20.0
        else return 120.0
        end
    end
    
    # Inicialización
    u0 = [0.0, 0.0] # Empieza en 0 grados, quieto
    current_control = 0.0
    
    prob = ODEProblem(dc_motor_physics!, u0, (0.0, dt_sim), [current_control])
    integrator = init(prob, Tsit5())
    
    # Arrays de datos
    times = 0.0:dt_sim:t_end
    pos_data = Float64[]
    vel_data = Float64[]
    ref_data = Float64[]
    ctrl_data = Float64[]
    
    # --- BUCLE DE SIMULACIÓN ---
    for t in times
        # 1. Obtener referencia actual
        setpoint = get_setpoint(t)
        
        # 2. Ejecutar PID (solo si toca por tiempo de muestreo)
        # El PID "ve" la posición actual del integrador (feedback)
        new_u = compute_pid(my_pid, setpoint, integrator.u[1], t)
        if new_u !== nothing
            current_control = new_u
        end
        
        # 3. Aplicar al motor (física)
        integrator.p[1] = current_control
        step!(integrator, dt_sim, true)
        
        # 4. Guardar datos
        push!(pos_data, integrator.u[1])
        push!(vel_data, integrator.u[2])
        push!(ref_data, setpoint)
        push!(ctrl_data, current_control)
    end
    
    # --- GRAFICACIÓN ---
    default(titlefont=font(10, "Computer Modern"), guidefont=font(9), tickfont=font(8))

    # Gráfica 1: Posición (Tracking)
    p1 = plot(times, ref_data, label="Setpoint (Referencia)", 
              linestyle=:dash, color=:red, linewidth=1.5,
              ylabel=L"Ángulo $\theta$ ($^\circ$)",
              title="A. Respuesta de Posición (Lazo Cerrado PID)")
    plot!(p1, times, pos_data, label="Posición Real", color=:blue, linewidth=2)

    # Gráfica 2: Velocidad
    p2 = plot(times, vel_data, label="Velocidad Angular", color=:green,
              ylabel=L"Velocidad $\omega$ ($^\circ/s$)",
              title="B. Dinámica de Velocidad")

    # Gráfica 3: Diagrama de Fase
    p3 = plot(pos_data, vel_data, label="Trayectoria de Estado", 
              color=:purple, alpha=0.8, linewidth=1,
              xlabel=L"Posición $\theta$ ($^\circ$)",
              ylabel=L"Velocidad $\omega$ ($^\circ/s$)",
              title="C. Diagrama de Fase (Convergencia de Error)")
    # Marcar los setpoints en el diagrama de fase
    scatter!(p3, [45, 90, 20, 120], [0,0,0,0], label="Puntos Objetivo", color=:red, markersize=4)

    savefig(p1, "pid_posicion.png")
    savefig(p2, "pid_velocidad.png")
    savefig(p3, "pid_fases.png")
    # Layout combinado
    l = @layout [a; b; c]
    p_final = plot(p1, p2, p3, layout=l, size=(800, 1100), dpi=300, margin=5Plots.mm)
    
    savefig(p_final, "pid_simulation_multistep.png")
    println("Simulación PID completada. Guardado: 'pid_simulation_multistep.png'")
end

run_pid_simulation()
