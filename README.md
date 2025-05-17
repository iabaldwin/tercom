# tercom

This project uses Bazel and raylib. Run the provided `initialise.sh` script to fetch dependencies and build raylib.

```
./initialise.sh
./thirdparty/bazelisk/bazel build //:tercom
```

On macOS the script now automatically downloads the darwin Bazelisk binary and builds raylib with CMake. Ensure that `git`, `cmake`, and either `wget` or `curl` are installed.
