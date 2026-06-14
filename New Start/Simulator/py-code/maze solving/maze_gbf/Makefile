# Makefile for Maze GBF Navigation System
#
# This Makefile compiles the maze navigation library and tests
# for host-based testing and validation.

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./inc
LDFLAGS = -lm

# Directories
SRC_DIR = src
INC_DIR = inc
TEST_DIR = test
BUILD_DIR = build
BIN_DIR = bin

# Source files
LIB_SRCS = $(SRC_DIR)/maze_gbf.c \
           $(SRC_DIR)/maze_graph.c \
           $(SRC_DIR)/maze_robot.c \
           $(SRC_DIR)/maze_utils.c

LIB_OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(LIB_SRCS))

# Test executables
TEST_BIN = $(BIN_DIR)/test_maze_gbf
INTEGRATION_TEST_BIN = $(BIN_DIR)/integration_test
HAL_TEST_BIN = $(BIN_DIR)/test_hal_integration

# Library file
LIB_NAME = libmaze_gbf.a
LIB_FILE = $(BIN_DIR)/$(LIB_NAME)

# Default target
.PHONY: all
all: dirs lib tests

# Create build directories
.PHONY: dirs
dirs:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

# Compile library
.PHONY: lib
lib: dirs $(LIB_FILE)

$(LIB_FILE): $(LIB_OBJS)
	@echo "Archiving library..."
	ar rcs $@ $^
	@echo "Library built: $@"

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile tests
.PHONY: tests
tests: lib $(TEST_BIN) $(INTEGRATION_TEST_BIN)

$(TEST_BIN): $(TEST_DIR)/test_maze_gbf.c $(LIB_FILE)
	@echo "Building unit tests..."
	$(CC) $(CFLAGS) $< -L$(BIN_DIR) -lmaze_gbf $(LDFLAGS) -o $@
	@echo "Unit tests built: $@"

$(INTEGRATION_TEST_BIN): $(TEST_DIR)/integration_test.c $(LIB_FILE)
	@echo "Building integration tests..."
	$(CC) $(CFLAGS) $< -L$(BIN_DIR) -lmaze_gbf $(LDFLAGS) -o $@
	@echo "Integration tests built: $@"

# Run tests
.PHONY: test
test: tests
	@echo "Running unit tests..."
	@./$(TEST_BIN)
	@echo ""
	@echo "Running integration tests..."
	@./$(INTEGRATION_TEST_BIN)

# Run unit tests only
.PHONY: test-unit
test-unit: tests
	@echo "Running unit tests..."
	@./$(TEST_BIN)

# Run integration tests only
.PHONY: test-integration
test-integration: tests
	@echo "Running integration tests..."
	@./$(INTEGRATION_TEST_BIN)

# Host-side HAL integration test (stubs firmware globals; no STM32 toolchain).
.PHONY: test-hal
test-hal: dirs $(HAL_TEST_BIN)
	@echo "Running HAL integration tests..."
	@./$(HAL_TEST_BIN)

$(HAL_TEST_BIN): $(TEST_DIR)/test_hal_integration.c $(LIB_SRCS)
	@echo "Building HAL integration tests..."
	$(CC) $(CFLAGS) -DMAZE_HAL_TEST -Istm-sample-code $^ $(LDFLAGS) -o $@
	@echo "HAL tests built: $@"

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR) $(BIN_DIR)
	@echo "Clean complete"

# Clean and rebuild
.PHONY: rebuild
rebuild: clean all

# Install library (for use in other projects)
.PHONY: install
install: lib
	@echo "Installing library..."
	@mkdir -p /usr/local/include/maze_gbf
	@cp $(INC_DIR)/*.h /usr/local/include/maze_gbf/
	@cp $(LIB_FILE) /usr/local/lib/
	@echo "Installation complete"

# Uninstall library
.PHONY: uninstall
uninstall:
	@echo "Uninstalling library..."
	@rm -rf /usr/local/include/maze_gbf
	@rm -f /usr/local/lib/$(LIB_NAME)
	@echo "Uninstall complete"

# Print help
.PHONY: help
help:
	@echo "Maze GBF Navigation System - Build Help"
	@echo ""
	@echo "Targets:"
	@echo "  all              - Build library and tests (default)"
	@echo "  lib              - Build library only"
	@echo "  tests            - Build tests only"
	@echo "  test             - Build and run all tests"
	@echo "  test-unit        - Build and run unit tests"
	@echo "  test-integration - Build and run integration tests"
	@echo "  clean            - Remove build artifacts"
	@echo "  rebuild          - Clean and build everything"
	@echo "  install          - Install library to system (requires sudo)"
	@echo "  uninstall        - Remove library from system (requires sudo)"
	@echo "  help             - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make              - Build everything"
	@echo "  make test         - Build and run tests"
	@echo "  make clean        - Clean build directory"
	@echo ""
	@echo "For STM32 compilation, use ARM GCC toolchain instead:"
	@echo "  arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb ..."

# Debug target - print variables
.PHONY: debug
debug:
	@echo "Source files: $(LIB_SRCS)"
	@echo "Object files: $(LIB_OBJS)"
	@echo "Library: $(LIB_FILE)"
	@echo "Test binary: $(TEST_BIN)"
	@echo "Integration test binary: $(INTEGRATION_TEST_BIN)"
