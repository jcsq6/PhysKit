from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import CMake, cmake_layout

class PhysKit(ConanFile):
    name = "physkit"
    version = "0.1.0"
    license = "MIT"
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires(
            "mp-units/2.5.0@mpusz/testing",
            options={
                "cxx_modules": True,
                "import_std": False,
                "std_format": True,
                # "no_crtp": True,
                # "contracts": "gsl-lite",
                # "freestanding": False,
            },
        )
        
        self.requires("eigen/3.4.0")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if can_run(self):
            cmake.ctest(cli_args=["--output-on-failure"])