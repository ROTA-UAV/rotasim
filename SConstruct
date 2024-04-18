#!/usr/bin/env python

env = SConscript("external/godot-cpp/SConstruct")

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

inc_dirs = ["src",
            "external/MAVSDK/install/include",
            "external/MAVSDK/install/include/mavsdk",
            "external/jsbsim/build/install/include/JSBSim"]

lib_dirs = ["external/MAVSDK/install/lib",
            "external/jsbsim/build/install/lib"]

libs = ["libmavsdk", "libjsbsim"]

env.Append(CPPPATH=inc_dirs)
env.Append(LIBPATH=lib_dirs)
env.Append(LIBS=libs)
sources = Glob("src/*.cpp") + Glob("src/**/*.cpp")

if env["platform"] == "macos":
    rotasim = env.SharedLibrary(
        "project/addons/rotasim/bin/librotasim.{}.{}.framework/librotasim.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    rotasim = env.SharedLibrary(
        "project/addons/rotasim/bin/librotasim{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(rotasim)

env.Tool("compilation_db")
cdb = env.CompilationDatabase("compile_commands.json")
Alias("cdb", cdb)

