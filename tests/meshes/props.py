import numpy as np
import trimesh

print ("heloo")

trimesh.util.attach_to_log()



mesh = trimesh.load('textured.dae',force='mesh')

print (trimesh.units.unit_conversion('meters', 'centimeters'))

mesh.is_watertight

print (mesh.center_mass*100)

print (mesh.moment_inertia*100)

f= open("prop.sdf","w+")
f.write("<sdf version=\"1.6\">\r\n")
f.write("   <model name=\"model_00\">\r\n")
f.write("       <pose>0 0 0.5 0 0 0</pose>\r\n")
f.write("       <link name=\"link_00\">\r\n")
f.write("       <pose>0.0 0 0 0 0 0</pose>\r\n")
f.write("       <collision name=\"collision_00\">\r\n")
f.write("           <pose>0.0 0 0 0 0 0</pose>\r\n")
f.write("           <geometry>\r\n")
f.write("               <sphere>\r\n")
f.write("                   <radius>0.5</radius>\r\n")
f.write("               </sphere>\r\n")
f.write("           </geometry>\r\n")
f.write("       </collision>\r\n")
f.write("       <visual name=\"visual_00\">\r\n")
f.write("           <geometry>\r\n")
f.write("               <sphere>\r\n")
f.write("                   <radius>0.5</radius>\r\n")
f.write("               </sphere>\r\n")
f.write("           </geometry>\r\n")
f.write("       </visual>\r\n")
f.write("       </link>\r\n")
f.write("   </model>\r\n")
f.write("</sdf>\r\n")
f.close()
