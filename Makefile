default: run

cmake:
	cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=/snap/clion/296/bin/ninja/linux/x64/ninja -G Ninja -S . -B cmake-build-debug
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=/snap/clion/296/bin/ninja/linux/x64/ninja -G Ninja -S . -B cmake-build-release

build: cmake
	cmake --build cmake-build-debug --target untitled -j 14

build_release: cmake
	cmake --build cmake-build-release --target untitled -j 14

run: build
	@echo "Running in debug mode...\n"
	@cmake-build-debug/untitled

run_release: build_release
	@echo "Running in release mode...\n"
	@cmake-build-release/untitled
clean_cmake:
	rm -r cmake-build-debug 2> /dev/null
	rm -r cmake-build-release 2> /dev/null



clean:
	rm -r cmake-build-debug/untitled 2> /dev/null
	rm -r ./cmake-build-debug/CMakeFiles/untitled.dir/ 2> /dev/null
	rm -r cmake-build-release/untitled 2> /dev/null
	rm -r ./cmake-build-release/CMakeFiles/untitled.dir/ 2> /dev/null
	echo ""

build_release:
	cmake --build cmake-build-release --target untitled -j 14
