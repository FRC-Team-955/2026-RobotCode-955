import usb_hid
import time
from adafruit_macropad import MacroPad
import board
import neopixel
from hid_gamepad import Gamepad

macropad = MacroPad()
gamepad = Gamepad(usb_hid.devices)

pixel_pin = board.NEOPIXEL
num_pixels = 12
ORDER = neopixel.RGB

def set_keys(on, color, dim, *indices):
    for i in indices:
        if (on):
            macropad.pixels[i] = color
        else:
            macropad.pixels[i] = tuple([rgb * dim for rgb in color])

class Button_Group:
    def __init__(self, inputIds, outputIds, color, dim):
        self.inputIds = inputIds
        self.outputIds = outputIds
        self.color = color
        self.dim = dim
        self.current = 0

    def handle_inputs(self, number):
        if number in self.inputIds:
            self.set(self.inputIds.index(number))

    def display_colors(self):
        self.set(self.current)

    def set(self, index):
        self.current = index
        outputIdsCopy = self.outputIds.copy()
        inputIdsCopy = self.inputIds.copy()
        set_keys(True, self.color, self.dim, inputIdsCopy.pop(index))
        gamepad.press_buttons(outputIdsCopy.pop(index))
        set_keys(False, self.color, self.dim, *inputIdsCopy)
        gamepad.release_buttons(*outputIdsCopy)

class OverrideButtons:
    def __init__(self, inputIds, outputIds, color, dim):
        self.inputIds = inputIds
        self.outputIds = outputIds
        self.toggles = [False for _ in inputIds]
        self.color = color
        self.dim = dim

    def handle_inputs(self, number):
        if number in self.inputIds:
            index = self.inputIds.index(number)
            self.toggles[index] = not self.toggles[index]
            self.set(index)

    def display_colors(self):
        # Update the colors
        for number in self.inputIds:
            index = self.inputIds.index(number)
            self.set(index)

    def set(self, index):
        set_keys(self.toggles[index], self.color, self.dim, self.inputIds[index])
        if self.toggles[index]:
            gamepad.press_buttons(self.outputIds[index])
        else:
            gamepad.release_buttons(self.outputIds[index])

class SimpleButtons:
    def __init__(self, inputIds, outputIds, color, dim):
        self.inputIds = inputIds
        self.outputIds = outputIds
        self.toggles = [False for _ in inputIds]
        self.color = color
        self.dim = dim

    def handle_inputs(self, number, pressed):
        if number in self.inputIds:
            index = self.inputIds.index(number)
            self.toggles[index] = pressed
            self.set(index)

    def display_colors(self):
        # Update the colors
        for number in self.inputIds:
            index = self.inputIds.index(number)
            self.set(index)

    def set(self, index):
        set_keys(self.toggles[index], self.color, self.dim, self.inputIds[index])
        if self.toggles[index]:
            gamepad.press_buttons(self.outputIds[index])
        else:
            gamepad.release_buttons(self.outputIds[index])

# reverse order so that l4 is the default
level = Button_Group([6, 7, 8, 11], [10, 9, 8, 7], (230, 100, 0), 0.02)
# front left default I guess
reefSides = Button_Group([3, 0, 1, 2, 4, 5], [1, 6, 5, 4, 2, 3], (150, 0, 10), 0.02)
# left default
localSide = Button_Group([9, 10], [11, 12], (60, 60, 60), 0.02)
overrides = OverrideButtons([0, 1, 2, 4], [1, 2, 3, 5], (10, 0, 150), 0.02)
overrideButtons = SimpleButtons([3, 5], [4, 6], (10, 0, 150), 0.02)

# Encoder switch
useReefSides = False
gamepad.release_buttons(13)
level.display_colors()
localSide.display_colors()
overrides.display_colors()
overrideButtons.display_colors()

while True:
    key_event = macropad.keys.events.get()
    macropad.encoder_switch_debounced.update()

    if macropad.encoder_switch_debounced.fell:
        print("Encoder switch pressed")
        useReefSides = not useReefSides
        if useReefSides:
            gamepad.press_buttons(13)
            reefSides.display_colors()
        else:
            gamepad.release_buttons(13)
            overrides.display_colors()
            overrideButtons.display_colors()

    if key_event:
        if key_event.pressed:
            print("Key pressed: {}".format(key_event.key_number))
        elif key_event.released:
            print("Key released: {}".format(key_event.key_number))

    if key_event:
        if key_event.pressed:
            level.handle_inputs(key_event.key_number)
            localSide.handle_inputs(key_event.key_number)
            if useReefSides:
                reefSides.handle_inputs(key_event.key_number)
            else:
                overrides.handle_inputs(key_event.key_number)
        if not useReefSides:
            overrideButtons.handle_inputs(key_event.key_number, key_event.pressed)
