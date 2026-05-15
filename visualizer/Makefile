.DEFAULT: help
.PHONY: help
help:
	@echo Use tab completion to view targets

.PHONY: build
build:
	cmake -B build -DCMAKE_PREFIX_PATH=$(CMAKE_PREFIX_PATH) && cmake --build build

.PHONY: run
run: build
	./build/visualizer

.PHONY: clean
clean:
	rm -rf build
