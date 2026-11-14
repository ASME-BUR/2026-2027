"""PlatformIO pre-build script: copy the vendored bur_msgs into extra_packages/.

micro_ros_platformio builds every ROS package under <project>/extra_packages/
into the firmware's message library. The single source of truth for the
messages is onboard/src/interfaces/bur_msgs, so it is copied here on every
build rather than committed twice. (A symlink would do the same job but does
not survive a Windows checkout.)

micro_ros_platformio only rebuilds its library when the library is missing, so
when the messages change this script also deletes the built library to force
a rebuild.
"""

import hashlib
import os
import shutil

Import("env")  # noqa: F821  (provided by PlatformIO's SCons)

PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
SRC = os.path.normpath(
    os.path.join(PROJECT_DIR, "..", "..", "src", "interfaces", "bur_msgs"))
DST = os.path.join(PROJECT_DIR, "extra_packages", "bur_msgs")
STAMP = os.path.join(PROJECT_DIR, "extra_packages", ".bur_msgs.sha1")
FILES = ("package.xml", "CMakeLists.txt")


def source_files():
    out = [os.path.join(SRC, f) for f in FILES]
    msg_dir = os.path.join(SRC, "msg")
    out += sorted(os.path.join(msg_dir, f) for f in os.listdir(msg_dir)
                  if f.endswith(".msg"))
    return out


def digest(paths):
    h = hashlib.sha1()
    for p in paths:
        h.update(os.path.relpath(p, SRC).replace(os.sep, "/").encode())
        with open(p, "rb") as f:
            h.update(f.read().replace(b"\r\n", b"\n"))
    return h.hexdigest()


paths = source_files()
new = digest(paths)
old = open(STAMP).read().strip() if os.path.isfile(STAMP) else None

if new != old or not os.path.isdir(DST):
    shutil.rmtree(DST, ignore_errors=True)
    for p in paths:
        rel = os.path.relpath(p, SRC)
        os.makedirs(os.path.dirname(os.path.join(DST, rel)), exist_ok=True)
        shutil.copyfile(p, os.path.join(DST, rel))
    with open(STAMP, "w") as f:
        f.write(new + "\n")

    libmicroros = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"),  # noqa: F821
                               env["PIOENV"], "micro_ros_platformio",  # noqa: F821
                               "libmicroros")
    if old is not None and os.path.isdir(libmicroros):
        print("bur_msgs changed: forcing a micro-ROS library rebuild")
        shutil.rmtree(libmicroros)
    print("bur_msgs synced into extra_packages/")
