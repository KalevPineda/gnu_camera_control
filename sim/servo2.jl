

import Pkg
# Asegúrate de tener instalados los paquetes
# Pkg.add(["Plots", "DifferentialEquations", "LaTeXStrings"])

using Plots
using DifferentialEquations
using LaTeXStrings # Para usar notación matemática en las leyendas

# --- PARÁMETROS FÍSICOS DEL SERVO (Modelo 2do Orden) ---
const ω_n = 12.0   # Frecuencia natural (rad/s)
const ζ = 0.70     # Coeficiente de amortiguamiento (Subamortiguado ideal)
const K = 1.0      # Ganancia

# Dinámica del sistema: x1 = posición, x2 = velocidad
function servo_dynamics!(du, u, p, t)
    theta, omega = u
    target_angle = p[1] # Referencia actual (PWM)
    
    # dθ/dt = ω
    du[1] = omega
    # dω/dt = ... (Ecuación masa-resorte-amortiguador)
    du[2] = (ω_n^2 * target_angle) - (2 * ζ * ω_n * omega) - (ω_n^2 * theta)
end

function run_simulation_and_plot()
    # Configuración de tiempos
    t_start = 0.0
    t_end = 12.0 # Simulamos 12 segundos para ver un par de ciclos
    dt = 0.01
    
    # Parámetros del paneo GSU
    angle_left = 125.0
    angle_right = 55.0
    scan_interval = 5.0 # Segundos entre cambios
    
    # Inicialización
    u0 = [angle_left, 0.0] # [Posición inicial, Velocidad 0]
    current_target = angle_left
    last_switch_time = 0.0
    
    # Arrays para almacenar datos
    times = t_start:dt:t_end
    pos_data = Float64[]
    vel_data = Float64[]
    ref_data = Float64[]
    
    # Configuración del problema diferencial
    prob = ODEProblem(servo_dynamics!, u0, (t_start, dt), [current_target])
    integrator = init(prob, Tsit5())
    
    # --- BUCLE DE SIMULACIÓN ---
    for t in times
        # Lógica Discreta del ESP32 (Máquina de Estados)
        if (t - last_switch_time) >= scan_interval
            if current_target == angle_left
                current_target = angle_right
            else
                current_target = angle_left
            end
            last_switch_time = t
        end
        
        # Actualizar referencia del integrador
        integrator.p[1] = current_target
        
        # Avanzar física
        step!(integrator, dt, true)
        
        # Guardar datos
        push!(pos_data, integrator.u[1]) # Posición
        push!(vel_data, integrator.u[2]) # Velocidad
        push!(ref_data, current_target)  # Referencia
    end

    # --- GENERACIÓN DE GRÁFICAS ---
    
    # Estilo general
    default(titlefont=font(10, "Computer Modern"), guidefont=font(9), tickfont=font(8))

    # 1. Gráfica de Respuesta Temporal (Posición)
    p1 = plot(times, ref_data, 
        label="Referencia (PWM)", 
        linestyle=:dash, color=:red, linewidth=1.5,
        ylabel=L"Posición $\theta$ ($^\circ$)",
        title="A. Respuesta Transitoria de Posición")
    plot!(p1, times, pos_data, 
        label="Trayectoria Real Servo", 
        color=:blue, linewidth=2)

    # 2. Gráfica de Velocidad Angular
    p2 = plot(times, vel_data,
        label="Velocidad Angular",
        color=:green, linewidth=1.5,
        ylabel=L"Velocidad $\omega$ ($^\circ/s$)",
        xlabel="Tiempo (s)",
        title="B. Perfil de Velocidad del Actuador")
    
    # 3. Diagrama de Fase (Posición vs Velocidad)
    # Muestra la estabilidad del sistema (ciclos límite o puntos de equilibrio)
    p3 = plot(pos_data, vel_data,
        label="Trayectoria de Estado",
        color=:purple, linewidth=1.5,
        xlabel=L"Posición $\theta$ ($^\circ$)",
        ylabel=L"Velocidad $\omega$ ($^\circ/s$)",
        title="C. Espacio de Fase",
        arrow=true) # Muestra flechas de dirección
    # Marcar los puntos de equilibrio (Atractores)
    scatter!(p3, [angle_left, angle_right], [0, 0], 
        label="Puntos de Equilibrio", 
        color=:red, markersize=5)

    # --- GUARDAR INDIVIDUALES ---
    savefig(p1, "gsu_posicion.png")
    savefig(p2, "gsu_velocidad.png")
    savefig(p3, "gsu_fase.png")

    # --- GRÁFICA COMBINADA (LAYOUT) ---
    # Creamos un layout: p1 arriba, p2 en medio, p3 abajo
    l = @layout [a; b; c]
    
    p_final = plot(p1, p2, p3, layout=l, size=(800, 1000), dpi=300, margin=5Plots.mm)
    
    savefig(p_final, "gsu_simulation_combined.png")
    println("Generación completa. Archivo guardado: 'gsu_simulation_combined.png'")
end

# Ejecutar
run_simulation_and_plot()

