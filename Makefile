default: build

prebuild_debug:
	cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=ninja -G Ninja -S . -B cmake-build-debug

prebuild:
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=ninja -G Ninja -S . -B cmake-build-release

build_debug:
	cmake --build cmake-build-debug --target gui -j 14
	cmake --build cmake-build-debug --target pc -j 14

build:
	cmake --build cmake-build-release --target gui -j 14
	cmake --build cmake-build-release --target pc -j 14

clean_cmake:
	rm -rf cmake-build-debug 2> /dev/null
	rm -rf cmake-build-release 2> /dev/null


clean_debug:
	rm -r cmake-build-debug/pc 2> /dev/null
	rm -r cmake-build-debug/gui 2> /dev/null

clean:
	rm -r cmake-build-release/pc 2> /dev/null
	rm -r cmake-build-release/gui 2> /dev/null

build_release:
	cmake --build cmake-build-release --target untitled -j 14
