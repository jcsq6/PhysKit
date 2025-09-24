# PhysKit
3D general purpose physics engine / simulation library

# Dependencies
Depends on:
- `mp-units`

# Build
With Conan, run:  
```sh
conan remote add conan-mpusz https://mpusz.jfrog.io/artifactory/api/conan/conan-oss
```

Ensure you have a compliant C++26 conan profile, and then run:  
```sh
conan build -pr <myprof|default> . -s build_type=<Release|Debug> --build=missing -u
```

# Contributing
See the following pages for dependency documentation:
- [mp-units](https://mpusz.github.io/mp-units/latest/)
- [Eigen](https://devdocs.io/eigen3/)