import numpy as np
import trimesh

print ("heloo")

trimesh.util.attach_to_log()



mesh = trimesh.load('textured.dae',force='mesh')

print trimesh.units.unit_conversion('meters', 'centimeters')

mesh.is_watertight

print mesh.center_mass*100

print mesh.moment_inertia*100
