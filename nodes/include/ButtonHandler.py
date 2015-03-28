#!/usr/bin/env python

class Button:
    # enum buttons
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    Black = 6
    Start = 7
    Center = 8


class ButtonHandler:
    def __init__(self, joy=None):
        self.joy = joy

    def is_active(self, button):
        return self.joy.buttons[button] == 1

    def a_active(self):
        return self.is_active(Button.A)

    def b_active(self):
        return self.is_active(Button.B)

    def x_active(self):
        return self.is_active(Button.X)

    def y_active(self):
        return self.is_active(Button.Y)

    def lb_active(self):
        return self.is_active(Button.LB)

    def rb_active(self):
        return self.is_active(Button.RB)

    def black_active(self):
        return self.is_active(Button.Black)

    def start_active(self):
        return self.is_active(Button.Start)

    def center_active(self):
        return self.is_active(Button.Center)

    def arrow_up_active(self):
        return self.joy.axes[7] == 1.0

    def arrow_down_active(self):
        return self.joy.axes[7] == -1.0

    def arrow_left_active(self):
        return self.joy.axes[6] == 1.0

    def arrow_right_active(self):
        return self.joy.axes[6] == -1.0

