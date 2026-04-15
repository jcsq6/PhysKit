from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain, CMakeDeps
from conan.errors import ConanInvalidConfiguration


class PhysKit(ConanFile):
    name = "physkit"
    version = "0.1.0"
    license = "MIT"
    settings = "os", "arch", "compiler", "build_type"

    options = {
        "build_tests": [True, False],
        "dev": [True, False],
        "physkit_modules": [True, False],
        "import_std": [True, False],
        "build_static": [True, False],
    }
    default_options = {
        "build_tests": True,
        "dev": True,
        "physkit_modules": False,
        "import_std": False,
        "build_static": False,
    }

    def configure(self):
        if self.settings.compiler == "gcc" and self.options.physkit_modules:
            raise ConanInvalidConfiguration(
                "C++ modules are not supported with GCC. "
                "Use Clang (-pr clang) or disable modules (-o '&:physkit_modules=False')."
            )

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["PHYSKIT_BUILD_TESTS"] = self.options.build_tests
        if self.options.dev:
            tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
            tc.cache_variables["CMAKE_VERIFY_INTERFACE_HEADER_SETS"] = (
                not self.options.import_std
            )

        tc.cache_variables["PHYSKIT_MODULES"] = bool(self.options.physkit_modules)
        tc.cache_variables["PHYSKIT_GRAPHICS_MODULES"] = bool(
            self.options.physkit_modules
        )
        if self.options.physkit_modules:
            tc.cache_variables["CMAKE_CXX_SCAN_FOR_MODULES"] = True
        if self.options.import_std:
            tc.cache_variables["CMAKE_CXX_MODULE_STD"] = True
            tc.cache_variables["CMAKE_EXPERIMENTAL_CXX_IMPORT_STD"] = (
                "d0edc3af-4c50-42ea-a356-e2862fe7a444"
            )
            
        tc.cache_variables["PHYSKIT_BUILD_STATIC"] = self.options.build_static

        tc.cache_variables["PHYSKIT_BUILD_STATIC"] = self.options.build_static

        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def requirements(self):
        self.requires(
            "mp-units/2.5.0",
            options={
                "cxx_modules": self.options.physkit_modules,
                "import_std": self.options.import_std,
                "std_format": True,
                "contracts": "none",
            },
        )

        self.requires("eigen/5.0.1")

        self.requires(
            "abseil/20250814.1"
        )  # TODO: update to latest release when available on conan

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
