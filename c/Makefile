# libosc001 — C99、外部依存なし(Linux では glibc のみ)
#   make        … libosc001.a と demo をビルド
#   make check  … オフラインテスト(モックトランスポート。デバイス不要)をビルドして実行

CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic
CXX     ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Wpedantic
LDLIBS  ?= -lm
BUILD   := build

LIB_OBJS := $(BUILD)/osc001_core.o $(BUILD)/transport_hidraw.o

.PHONY: all check clean
all: $(BUILD)/libosc001.a $(BUILD)/demo_capture $(BUILD)/lock_check \
	$(BUILD)/demo_capture_cpp

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/osc001_core.o: src/osc001_core.c include/osc001.h | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude -c $< -o $@

$(BUILD)/transport_hidraw.o: src/transport_hidraw.c include/osc001.h | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude -c $< -o $@

$(BUILD)/libosc001.a: $(LIB_OBJS)
	$(AR) rcs $@ $^

$(BUILD)/demo_capture: examples/demo_capture.c $(BUILD)/libosc001.a | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude $< $(BUILD)/libosc001.a $(LDLIBS) -o $@

$(BUILD)/lock_check: examples/lock_check.c $(BUILD)/libosc001.a | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude $< $(BUILD)/libosc001.a $(LDLIBS) -o $@

$(BUILD)/test_osc001: test/test_osc001.c test/mock_transport.h $(BUILD)/libosc001.a | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude $< $(BUILD)/libosc001.a $(LDLIBS) -o $@

$(BUILD)/demo_capture_cpp: examples/demo_capture.cpp include/osc001.hpp $(BUILD)/libosc001.a | $(BUILD)
	$(CXX) $(CXXFLAGS) -Iinclude $< $(BUILD)/libosc001.a $(LDLIBS) -o $@

$(BUILD)/test_osc001_cpp: test/test_osc001_cpp.cpp include/osc001.hpp test/mock_transport.h $(BUILD)/libosc001.a | $(BUILD)
	$(CXX) $(CXXFLAGS) -Iinclude $< $(BUILD)/libosc001.a $(LDLIBS) -o $@

check: $(BUILD)/test_osc001 $(BUILD)/test_osc001_cpp
	./$(BUILD)/test_osc001
	./$(BUILD)/test_osc001_cpp

clean:
	rm -rf $(BUILD)
