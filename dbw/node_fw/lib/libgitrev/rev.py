import subprocess

def get_git_specifier():
    rev_stat = subprocess.run(['git rev-parse --short HEAD --'], shell=True, capture_output=True)
    if rev_stat.returncode != 0:
        print("Error describing git revision.")
        exit(-1)

    rev = rev_stat.stdout.rstrip()
    print(f"-DBUILD_GIT_REV=0x{str(rev, 'utf-8').upper()}U")

get_git_specifier()
