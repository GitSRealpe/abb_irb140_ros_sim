import numpy as np
import trimesh
from math import floor

print ("heloo")

trimesh.util.attach_to_log()



mesh = trimesh.load('cafe_can/textured.dae',force='mesh')

print (trimesh.units.unit_conversion('meters', 'centimeters'))

mesh.is_watertight

print (mesh.center_mass*100)

print (mesh.moment_inertia*100)
i=1
f= open("cafe_can/model.sdf","w+")
f.write("<sdf version=\"1.6\">\r\n")
f.write("   <model name=\"cafe_can\">\r\n")
f.write("       <pose>0 0 0.5 0 0 0</pose>\r\n")
f.write("       <link name=\"link_00\">\r\n")
f.write("       <pose>0.0 0 0 0 0 0</pose>\r\n")
f.write("       <visual name=\"visual_00\">\r\n")
f.write("           <geometry>\r\n")
f.write("               <mesh><uri>file://cafe_can/textured.dae</uri></mesh>\r\n")
f.write("           </geometry>\r\n")
f.write("       </visual>\r\n")
f.write("       <collision name=\"collision\">\r\n")
f.write("           <geometry>\r\n")
f.write("               <mesh><uri>file://cafe_can/textured.dae</uri></mesh>\r\n")
f.write("           </geometry>\r\n")
f.write("       </collision>\r\n")
f.write("       <inertial>\r\n")
f.write("           <pose> "+str(round(mesh.center_mass[0],3))+" "
                            +str(round(mesh.center_mass[1],3))+" "
                            +str(round(mesh.center_mass[2],3))+" 0 0 0</pose>\r\n")
f.write("           <inertia>\r\n")
f.write("               <ixx>"+str(round(mesh.moment_inertia[0][0],7)*100)+"</ixx>\r\n")
f.write("               <ixy>"+str(round(mesh.moment_inertia[0][1],7)*100)+"</ixy>\r\n")
f.write("               <ixz>"+str(round(mesh.moment_inertia[0][2],7)*100)+"</ixz>\r\n")
f.write("               <iyy>"+str(round(mesh.moment_inertia[1][1],7)*100)+"</iyy>\r\n")
f.write("               <iyz>"+str(round(mesh.moment_inertia[1][2],7)*100)+"</iyz>\r\n")
f.write("               <izz>"+str(round(mesh.moment_inertia[2][2],7)*100)+"</izz>\r\n")
f.write("           </inertia>\r\n")
f.write("       </inertial>\r\n")
f.write("       </link>\r\n")
f.write("   </model>\r\n")
f.write("</sdf>\r\n")
f.close()
