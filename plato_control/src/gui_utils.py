import pygame
import sys

BLACK = (0,   0,   0)
WHITE = (255, 255, 255)
RED = (200, 0, 0)
GREEN = (0, 200, 0)
BRIGHT_RED = (255, 0, 0)
BRIGHT_GREEN = (0, 255, 0)


class Slider():
    """ Pygame slider class:

    Attributes:
        x: X position coordinate
        y: Y position coordinate
        width: Slider width
        value: initial slider value
        max: maximal slider value
    """

    def __init__(self, x, y, width, value=50, max=100):
        self.screen = pygame.display.get_surface()
        self.value = value
        self.max = max

        self.sliderRect = pygame.Rect(x, y, width, 20)
        self.squareRect = pygame.Rect(x, y, width / 2, 20)

        if pygame.font:
            self.font = pygame.font.Font(None, 22)

    def update(self):
        pygame.draw.rect(self.screen, (100, 100, 255), self.sliderRect)
        self.squareRect.w = self.value / float(self.max) * self.sliderRect.w
        pygame.draw.rect(self.screen, (255, 0, 0), self.squareRect)

        if pygame.font:
            self.textDisp = self.font.render(
                str(self.value), 1, (255, 255, 255))

        self.textRect = self.textDisp.get_rect(
            centerx=self.sliderRect.x + self.sliderRect.w / 2, centery=self.sliderRect.y + 11)
        self.screen.blit(self.textDisp, self.textRect)

    def onSlider(self, (x, y)):
        if x >= self.getX() and x <= (self.getX() + self.getWidth()) and y >= self.getY() and y <= (self.getY() + 20):
            return True
        else:
            return False

    def getX(self):
        return self.sliderRect.x

    def getY(self):
        return self.sliderRect.y

    def getWidth(self):
        return self.sliderRect.w

    def getValue(self):
        return self.value

    def setValueByMousePos(self, x):
        if x < self.sliderRect.x:
            self.value = 0
        elif x > (self.sliderRect.x + self.sliderRect.w):
            self.value = self.max
        else:
            self.value = int((x - self.sliderRect.x) /
                             float(self.sliderRect.w) * self.max)

    def setValueByNumber(self, value):
        self.value = value


class TextPrint(object):
    """ Text printed on pygame screen:
    """

    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def myprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


class JoystickGui(object):
    """ GUI for controlling robot using joystick:

    Attributes:
        windows_width: pygame screen width
        windows_height: pygame screen height
        caption: caption shown on pygame window
    """

    def __init__(self, window_width=500, window_height=700, caption='Robot joypad control'):
        pygame.init()
        self.screen = pygame.display.set_mode([window_width, window_height])
        pygame.display.set_caption(caption)
        self.clock = pygame.time.Clock()
        pygame.joystick.init()
        self.textPrint = TextPrint()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joy_name = self.joystick.get_name()
        self.joy_axes = self.joystick.get_numaxes()
        self.joy_buttons = self.joystick.get_numbuttons()
        self.joy_hats = self.joystick.get_numhats()
        self.buttons_pressed = [0] * self.joy_buttons
        self.axes_pressed = [0] * self.joy_axes
        self.hats_pressed = [(0, 0) for i in range(self.joy_hats)]

    def get_pressed_keys(self):
        return self.buttons_pressed, self.axes_pressed, self.hats_pressed

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
        self.screen.fill(WHITE)
        self.textPrint.reset()

        self.textPrint.myprint(
            self.screen, "Joystick name: {}".format(self.joy_name))
        self.textPrint.myprint(
            self.screen, "Number of axes: {}".format(self.joy_axes))
        self.textPrint.indent()
        self.textPrint.myprint(
            self.screen, "Number of buttons: {}".format(self.joy_buttons))
        self.textPrint.myprint(
            self.screen, "Number of hats: {}".format(self.joy_hats))

        for i in range(self.joy_axes):
            self.axes_pressed[i] = self.joystick.get_axis(i)
            self.textPrint.myprint(
                self.screen, "Axis {} value: {:>6.3f}".format(i, self.axes_pressed[i]))
        self.textPrint.unindent()

        self.textPrint.indent()

        for i in range(self.joy_buttons):
            self.buttons_pressed[i] = self.joystick.get_button(i)
            self.textPrint.myprint(
                self.screen, "Button {:>2} value: {}".format(i, self.buttons_pressed[i]))
        self.textPrint.unindent()
        self.textPrint.indent()

        for i in range(self.joy_hats):
            self.hats_pressed[i] = self.joystick.get_hat(i)
            self.textPrint.myprint(self.screen, "Hat {} value: {}".format(
                i, str(self.hats_pressed[i])))

        pygame.display.flip()
        self.clock.tick(60)

    def quit_gui(self):
        pygame.quit()


class KeyboardGui(object):
    """ GUI for controlling robot using keyboard:

    Attributes:
        windows_width: pygame screen width
        windows_height: pygame screen height
        caption: caption shown on pygame window
    """

    def __init__(self, window_width=640, window_height=480, caption='Robot keyboard control'):
        self.screen_color = BLACK
        self.arrow_buttons = RED
        self.arrow_buttons_active = BRIGHT_RED
        self.arrow_buttons_text_color = WHITE
        self.arrow_buttons_width = 100
        self.arrow_buttons_height = 50
        self.arrow_buttons_offset = 25
        self.arrow_buttons_x = 50
        self.arrow_buttons_y = 100
        self.button_text_shift_x = 20
        self.button_text_shift_y = 10

        pygame.init()
        self.font = pygame.font.SysFont('Arial', 25)
        self.screen = pygame.display.set_mode((window_width, window_height))
        self.screen.fill(self.screen_color)
        pygame.display.set_caption(caption)
        self.clock = pygame.time.Clock()

        self.slider_linear_pressed = False
        self.slider_angular_pressed = False

        a_pos = (self.arrow_buttons_x, self.arrow_buttons_y +
                 self.arrow_buttons_offset + self.arrow_buttons_height)
        s_pos = (self.arrow_buttons_x + self.arrow_buttons_offset + self.arrow_buttons_width,
                 self.arrow_buttons_y + self.arrow_buttons_offset + self.arrow_buttons_height)
        d_pos = (self.arrow_buttons_x + 2 * self.arrow_buttons_offset + 2 * self.arrow_buttons_width,
                 self.arrow_buttons_y + self.arrow_buttons_offset + self.arrow_buttons_height)
        w_pos = (self.arrow_buttons_x + self.arrow_buttons_offset +
                 self.arrow_buttons_width, self.arrow_buttons_y)

        self.a_button = (a_pos[0], a_pos[1], self.arrow_buttons_width,
                         self.arrow_buttons_height)
        self.s_button = (s_pos[0], s_pos[1], self.arrow_buttons_width,
                         self.arrow_buttons_height)
        self.d_button = (d_pos[0], d_pos[1], self.arrow_buttons_width,
                         self.arrow_buttons_height)
        self.w_button = (w_pos[0], w_pos[1], self.arrow_buttons_width,
                         self.arrow_buttons_height)
        self.slider_speed_linear = Slider(a_pos[0], s_pos[0] + 200, 500, 100)
        self.slider_speed_angular = Slider(a_pos[0], s_pos[0] + 250, 500, 100)
        self.buttons_pressed = [0] * 4
        self.mouse_pressed = [0] * 4
        self.slider_value_linear = 50
        self.slider_value_angular = 50

    def get_pressed_keys(self):
        return self.buttons_pressed, self.mouse_pressed, [self.slider_value_linear, self.slider_value_angular]

    def update(self):
        key_pressed = pygame.key.get_pressed()
        mouse_pressed = pygame.mouse.get_pressed()

        ctrl_pressed = key_pressed[pygame.K_RCTRL] or key_pressed[pygame.K_LCTRL]
        c_pressed = key_pressed[pygame.K_c]

        up_pressed = key_pressed[pygame.K_w] or key_pressed[pygame.K_UP]
        down_pressed = key_pressed[pygame.K_s] or key_pressed[pygame.K_DOWN]
        left_pressed = key_pressed[pygame.K_a] or key_pressed[pygame.K_LEFT]
        right_pressed = key_pressed[pygame.K_d] or key_pressed[pygame.K_RIGHT]
        q_pressed = key_pressed[pygame.K_q]
        e_pressed = key_pressed[pygame.K_e]
        z_pressed = key_pressed[pygame.K_z]
        x_pressed = key_pressed[pygame.K_x]

        if ctrl_pressed and c_pressed:
            sys.exit(0)

        left_clicked = self.button(u'\u25C0 A', self.a_button,
                                   self.arrow_buttons, self.arrow_buttons_active)
        down_clicked = self.button(u'\u25BC S', self.s_button,
                                   self.arrow_buttons, self.arrow_buttons_active)
        right_clicked = self.button(u'\u25B6 D', self.d_button,
                                    self.arrow_buttons, self.arrow_buttons_active)
        up_clicked = self.button(u'\u25B2 W', self.w_button,
                                 self.arrow_buttons, self.arrow_buttons_active)

        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (key_pressed[pygame.K_F4] and key_pressed[pygame.K_LALT]):
                sys.exit(0)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mse = pygame.mouse.get_pos()

                if self.slider_speed_linear.onSlider(mse):
                    self.slider_speed_linear.setValueByMousePos(mse[0])
                    self.slider_linear_pressed = True

                if self.slider_speed_angular.onSlider(mse):
                    self.slider_speed_angular.setValueByMousePos(mse[0])
                    self.slider_angular_pressed = True

            elif event.type == pygame.MOUSEMOTION:
                mse = pygame.mouse.get_pos()

                pygame.mouse.set_cursor(*pygame.cursors.arrow)

                if self.slider_speed_linear.onSlider(mse):
                    pygame.mouse.set_cursor(*pygame.cursors.tri_left)

                if self.slider_speed_angular.onSlider(mse):
                    pygame.mouse.set_cursor(*pygame.cursors.tri_left)

                if self.slider_linear_pressed and mouse_pressed[0]:
                    pygame.mouse.set_cursor(*pygame.cursors.tri_left)
                    self.slider_speed_linear.setValueByMousePos(mse[0])

                if self.slider_angular_pressed and mouse_pressed[0]:
                    pygame.mouse.set_cursor(*pygame.cursors.tri_left)
                    self.slider_speed_angular.setValueByMousePos(mse[0])

            elif event.type == pygame.MOUSEBUTTONUP:
                if self.slider_linear_pressed:
                    self.slider_linear_pressed = False

                if self.slider_angular_pressed:
                    self.slider_angular_pressed = False

        if e_pressed:
            if self.slider_speed_linear.getValue() < 100:
                self.slider_speed_linear.setValueByNumber(
                    self.slider_speed_linear.getValue() + 1)
        if q_pressed:
            if self.slider_speed_linear.getValue() > 0:
                self.slider_speed_linear.setValueByNumber(
                    self.slider_speed_linear.getValue() - 1)
        if x_pressed:
            if self.slider_speed_angular.getValue() < 100:
                self.slider_speed_angular.setValueByNumber(
                    self.slider_speed_angular.getValue() + 1)
        if z_pressed:
            if self.slider_speed_angular.getValue() > 0:
                self.slider_speed_angular.setValueByNumber(
                    self.slider_speed_angular.getValue() - 1)
        self.buttons_pressed = [up_pressed,
                                down_pressed, left_pressed, right_pressed]
        self.mouse_pressed = [up_clicked,
                              down_clicked, left_clicked, right_clicked]
        self.slider_value_linear = self.slider_speed_linear.getValue()
        self.slider_value_angular = self.slider_speed_angular.getValue()
        self.slider_speed_linear.update()
        self.slider_speed_angular.update()
        pygame.display.flip()
        self.clock.tick(60)

    def button(self, msg, button_pos, passive_color, active_color):
        mouse = pygame.mouse.get_pos()
        click = pygame.mouse.get_pressed()
        result = False

        if button_pos[0] + self.arrow_buttons_width > mouse[0] > button_pos[0] and \
                button_pos[1] + self.arrow_buttons_height > mouse[1] > button_pos[1]:
            if click[0]:
                pygame.draw.rect(self.screen, active_color, button_pos)
                result = True
            else:
                pygame.draw.rect(self.screen, passive_color, button_pos)
        else:
            pygame.draw.rect(self.screen, passive_color, button_pos)

        self.screen.blit(self.font.render(msg, True, self.arrow_buttons_text_color),
                         (button_pos[0] + self.button_text_shift_x, button_pos[1] + self.button_text_shift_y))
        return result

    def quit_gui(self):
        pygame.quit()
