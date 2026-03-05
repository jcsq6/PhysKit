from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain, CMakeDeps

class PhysKit(ConanFile):
    name = "physkit"
    version = "0.1.0"
    license = "MIT"
    settings = "os", "arch", "compiler", "build_type"
    
    options = {"build_tests": [True, False], "dev": [True, False]}
    default_options = {"build_tests": True, "dev": True}
    
    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["PHYSKIT_BUILD_TESTS"] = self.options.build_tests
        if self.options.dev:
            tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
            tc.variables["CMAKE_VERIFY_INTERFACE_HEADER_SETS"] = True
        tc.generate()
        
        deps = CMakeDeps(self)
        deps.generate()
    
    def requirements(self):
        self.requires(
            "mp-units/2.5.0@mpusz/testing",
            options={
                "cxx_modules": False,
                "import_std": False,
                "std_format": True,
                # "no_crtp": True,
                # "contracts": "gsl-lite",
                # "freestanding": False,
            },
        )
        
        self.requires("eigen/3.4.0")
        
        self.requires("abseil/20250814.1") # TODO: update to latest release when available on conan
        
        if self.options.build_tests:
            self.requires("glfw/3.4")
            self.requires("glaze/7.0.2")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if can_run(self):
            cmake.ctest(cli_args=["--output-on-failure"])