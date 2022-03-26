Import("env")

print("Generating CAN files.....")
env.Execute("make -C ../../can dbw_files")

print("Current build targets", list(map(str, BUILD_TARGETS)))

