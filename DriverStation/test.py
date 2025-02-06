import pygame
from pygame.locals import *
import imgui
from imgui.integrations.pygame import PygameRenderer
import csv
from datetime import datetime
import OpenGL.GL as gl

# Initialize Pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((1000, 800), OPENGL | DOUBLEBUF | RESIZABLE)
pygame.display.set_caption("Driver Control Station")

# Initialize OpenGL
pygame.display.gl_set_attribute(GL_DEPTH_SIZE, 24)
pygame.display.gl_set_attribute(GL_STENCIL_SIZE, 8)
pygame.display.gl_set_attribute(GL_CONTEXT_PROFILE_MASK, GL_CONTEXT_PROFILE_CORE)
pygame.display.gl_set_attribute(GL_CONTEXT_MAJOR_VERSION, 3)
pygame.display.gl_set_attribute(GL_CONTEXT_MINOR_VERSION, 3)

# Initialize ImGui
imgui.create_context()
io = imgui.get_io()
io.display_size = screen.get_width(), screen.get_height()
io.display_fb_scale = 1, 1
renderer = PygameRenderer()


# Telemetry values
telemetry_values = {
    "Battery Voltage": 0.0,
    "Deployment Height": 0.0,
    "Intake Speed": 0.0,
    "Dump Speed": 0.0
}

# Console log
console_log = []

# CSV file path
csv_file_path = datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".csv"

# Update telemetry values
def update_telemetry_values():
    for key in telemetry_values:
        telemetry_values[key] += 0.1

# Write to CSV
def write_to_csv(filename, data):
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)
# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == VIDEORESIZE:
            screen = pygame.display.set_mode((event.w, event.h), OPENGL | DOUBLEBUF | RESIZABLE)
            io.display_size = event.w, event.h
            io.display_fb_scale = 1, 1
            renderer.resize(event.w, event.h)

        renderer.process_event(event)
   
    update_telemetry_values()

    

    imgui.new_frame()

    imgui.begin("Telemetry Data")
    for key, value in telemetry_values.items():
        imgui.text(f"{key}: {value}")
    imgui.end()

    imgui.begin("Console Log")
    for log_message in console_log:
        imgui.text(log_message)
    imgui.end()

    imgui.begin("Control Buttons")
    if imgui.button("Reset Robot"):
        message = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Reset Robot"
        console_log.append(message)
        write_to_csv(csv_file_path, [message])

    imgui.same_line()

    if imgui.button("Deploy Intake"):
        message = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Deploy Intake"
        console_log.append(message)
        write_to_csv(csv_file_path, [message])

    imgui.same_line()

    if imgui.button("Switch Camera"):
        message = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Switch Camera"
        console_log.append(message)
        write_to_csv(csv_file_path, [message])

    imgui.same_line()

    if imgui.button("Zero Deploy Encoder"):
        message = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Zero Deploy Encoder"
        console_log.append(message)
        write_to_csv(csv_file_path, [message])

    imgui.end()
    
    #This line slows down everything by alot for whatever reason
    #pygame.event.clear()
    imgui.render()
    
    gl.glClearColor(0, 0, 0, 1)
    gl.glClear(gl.GL_COLOR_BUFFER_BIT)
    renderer.render(imgui.get_draw_data())
    renderer.process_event(event)
    

    pygame.display.flip()
    
    pygame.event.pump()

# Shutdown
renderer.shutdown()
imgui.destroy_context()
pygame.quit()