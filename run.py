import subprocess
proc = subprocess.run(["meshlabserver", "-i", "./"+dir+"/textured.dae", "-o", "./"+dir+"/collision.dae", "-s", "simp.mlx"])
proc.returncode
