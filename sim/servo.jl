
using Plots
using DifferentialEquations

# --- PARÁMETROS FÍSICOS DEL SERVO (Modelo de 2do Orden) ---
# Un servo SG90 no es instantáneo, tiene inercia.
const ω_n = 15.0  # Frecuencia natural (rad/s) - Velocidad de respuesta
const ζ = 0.75    # Coeficiente de amortiguamiento (sin sobreimpulso excesivo)
const K = 1.0     # Ganancia DC (entrada grados -> salida grados)

# Función de transferencia en espacio de estados:
# x1 = posición, x2 = velocidad
function servo_dynamics!(du, u, p, t)
    theta, omega = u
    target_angle = p[1] # La referencia actual del ESP32
    
    # Ecuación diferencial: d²θ/dt² + 2ζωn(dθ/dt) + ωn²θ = ωn² * Referencia
    du[1] = omega
    du[2] = (ω_n^2 * target_angle) - (2 * ζ * ω_n * omega) - (ω_n^2 * theta)
end

# --- LÓGICA DE CONTROL DEL ESP32 (REPLICADA) ---
# Mapeo PWM del código C a Grados (Aprox)
# Left (760) ~ 125°, Right (469) ~ 55°
const ANGLE_LEFT = 125.0
const ANGLE_RIGHT = 55.0
const SCAN_INTERVAL = 5.0 # Segundos

function simulate_gsu_system()
    # Tiempo total de simulación
    t_end = 15.0
    dt = 0.01
    times = 0:dt:t_end
    
    # Estado inicial del servo
    u0 = [ANGLE_LEFT, 0.0] # [Posición inicial, Velocidad inicial]
    
    # Arreglos para guardar historia
    pos_history = Float64[]
    ref_history = Float64[]
    time_history = Float64[]
    
    # Variables de estado del "Microcontrolador"
    current_target = ANGLE_LEFT
    last_switch_time = 0.0
    
    # Inicializar integrador
    # Usamos un integrador paso a paso para inyectar la lógica discreta
    prob = ODEProblem(servo_dynamics!, u0, (0.0, dt), [current_target])
    integrator = init(prob, Tsit5())

    for t in times
        # --- LÓGICA DEL ESP32 ---
        # Si pasaron 5 segundos, cambiar estado (Modo AUTO)
        if (t - last_switch_time) >= SCAN_INTERVAL
            if current_target == ANGLE_LEFT
                current_target = ANGLE_RIGHT
            else
                current_target = ANGLE_LEFT
            end
            last_switch_time = t
        end
        
        # Actualizar la referencia en la ecuación diferencial
        integrator.p[1] = current_target
        
        # Avanzar la física del motor un paso
        step!(integrator, dt, true)
        
        # Guardar datos
        push!(time_history, t)
        push!(pos_history, integrator.u[1]) # Posición real del motor
        push!(ref_history, current_target)  # Referencia del ESP32
    end
    
    # --- GRAFICAR ---
    plot(time_history, ref_history, 
         label="Señal PWM (Referencia ESP32)", 
         linestyle=:dash, color=:red, linewidth=2,
         xlabel="Tiempo (s)", ylabel="Ángulo (Grados)",
         title="Simulación: Respuesta Dinámica GSU Thermal Cam",
         legend=:topright)
         
    plot!(time_history, pos_history, 
          label="Posición Real Servo (Dinámica 2do Orden)", 
          color=:blue, linewidth=2)
          
    savefig("gsu_servo_simulation.png")
    println("Simulación completada. Gráfico guardado como 'gsu_servo_simulation.png'")
end

# Ejecutar simulación
simulate_gsu_system()
