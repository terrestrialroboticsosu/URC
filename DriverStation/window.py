import pygame
from pygame.locals import *
import imgui
from imgui.integrations.pygame import PygameRenderer
import OpenGL.GL as gl

DEFAULT_WINDOW_SIZE = (1000, 800)
WINDOW_TITLE = "Driver Station"
DISPLAY_MODE_OPTIONS = OPENGL | DOUBLEBUF | RESIZABLE
FONT_SIZE = 18


class Window:
    def __init__(self):
        self.window_size = DEFAULT_WINDOW_SIZE
        # Setup Window
        pygame.init()
        self.screen = pygame.display.set_mode(
            self.window_size, DISPLAY_MODE_OPTIONS)
        pygame.display.set_caption(WINDOW_TITLE)
        print("Window created")

        # Setup OpenGL
        pygame.display.gl_set_attribute(GL_DEPTH_SIZE, 24)
        pygame.display.gl_set_attribute(GL_STENCIL_SIZE, 8)
        pygame.display.gl_set_attribute(
            GL_CONTEXT_PROFILE_MASK, GL_CONTEXT_PROFILE_CORE)
        pygame.display.gl_set_attribute(GL_CONTEXT_MAJOR_VERSION, 3)
        pygame.display.gl_set_attribute(GL_CONTEXT_MINOR_VERSION, 3)
        print("OpenGL initialized")

        # Setup Imgui
        imgui.create_context()
        self.io = imgui.get_io()
        self.io.display_size = self.window_size
        self.io.display_fb_scale = 1, 1
        self.renderer = PygameRenderer()
        self.font = self.io.fonts.add_font_from_file_ttf(
            "Inconsolata-Regular.ttf", FONT_SIZE,
        )
        self.renderer.refresh_font_texture()
        print("ImGUI initialized")

    def process_events(self, state):
        for event in pygame.event.get():
            self.renderer.process_event(event)

            if event.type == QUIT:
                print("User closed window... shutting down!")
                pygame.display.set_mode(self.window_size, flags=pygame.HIDDEN)
                state.shutdown()
            elif event.type == VIDEORESIZE:
                self.window_size = event.w, event.h
                self.screen = pygame.display.set_mode(
                    self.window_size, DISPLAY_MODE_OPTIONS)
                self.io.display_size = self.window_size

                print(f"Resize to ({self.window_size})")

        self.renderer.process_inputs()

    def render(self, state, gui):
        imgui.new_frame()
        with imgui.font(self.font):
            gui.render(state, self.window_size)
        imgui.render()

        gl.glClearColor(0, 0, 0, 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        self.renderer.render(imgui.get_draw_data())
        pygame.display.flip()
