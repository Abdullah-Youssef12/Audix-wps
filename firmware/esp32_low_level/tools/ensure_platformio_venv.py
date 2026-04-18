Import("env")

import os
import subprocess
import venv


def _venv_python_path(venv_dir):
    if os.name == "nt":
        return os.path.join(venv_dir, "Scripts", "python.exe")
    return os.path.join(venv_dir, "bin", "python3")


project_dir = env.subst("$PROJECT_DIR")
venv_dir = os.path.join(project_dir, ".pio", "platformio-python-venv")
venv_python = _venv_python_path(venv_dir)

if not os.path.isfile(venv_python):
    print("Creating project-local PlatformIO Python venv: {}".format(venv_dir))
    try:
        venv.EnvBuilder(with_pip=True).create(venv_dir)
    except Exception as exc:
        raise RuntimeError(
            "Failed to create the project-local Python venv at {}. "
            "Install python3-venv (or the platform equivalent) and retry.".format(venv_dir)
        ) from exc

def _ensure_package_installed(package_spec):
    package_name = package_spec.split("==")[0]
    freeze_output = subprocess.check_output(
        [venv_python, "-m", "pip", "freeze"],
        text=True,
    )
    installed_packages = [line.split("==")[0].strip() for line in freeze_output.splitlines() if line.strip()]
    if package_name not in installed_packages:
        print("Installing {} into project-local PlatformIO Python".format(package_spec))
        subprocess.check_call([venv_python, "-m", "pip", "install", package_spec])


_ensure_package_installed("pyserial")
env.Replace(PYTHONEXE=venv_python)
print("Using project-local PlatformIO Python: {}".format(venv_python))
