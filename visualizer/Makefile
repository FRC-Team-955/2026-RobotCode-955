.DEFAULT: help
.PHONY: help
help:
	@echo Use tab completion to view targets

.PHONY: build
build:
	cmake -B build && cmake --build build

.PHONY: run
run: build
	./build/visualizer
