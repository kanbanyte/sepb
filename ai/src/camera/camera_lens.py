from enum import Enum

'''
Enum representing logical lenses of the camera.
The logical lenses are inverse with the physical lenses if the camera is flipped.
'''
LogicalLens = Enum("LogicalLens", ["LEFT", "RIGHT"])
