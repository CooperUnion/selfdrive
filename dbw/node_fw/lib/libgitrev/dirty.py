import subprocess

def get_git_specifier():
    dirt_stat = subprocess.run(['git diff-index --quiet HEAD --'], shell=True)
    if dirt_stat.returncode != 0 and dirt_stat.returncode != 1:
        print("Error describing git cleanliness.")
        exit(-1)

    print(f"-DBUILD_GIT_REV_DIRTY={dirt_stat.returncode}")

get_git_specifier()
