# ============================================================================
#  其它模块引用本库（推荐，先 make lib）：
#    #include "hc_sr04p.h"
#    g++ -std=c++17 -O2 -I. your_app.cpp libhcsr04p.a -o your_app
#  也可以只把 hc_sr04p.cpp 加进你的源文件列表，不需要编静态库。
#  注意：hc_sr04p_main.cpp 只是命令行演示，含 main()，不要编进你的程序。
# ============================================================================

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
AR       ?= ar
ARFLAGS  := rcs

HEADER   := hc_sr04p.h
LIB      := libhcsr04p.a
TARGET   := hc_sr04p
LIB_OBJ  := hc_sr04p.o
CLI_OBJ  := hc_sr04p_main.o

all: $(TARGET)

lib: $(LIB)

$(LIB_OBJ): hc_sr04p.cpp $(HEADER)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(CLI_OBJ): hc_sr04p_main.cpp $(HEADER)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(LIB): $(LIB_OBJ)
	$(AR) $(ARFLAGS) $@ $^

$(TARGET): $(CLI_OBJ) $(LIB)
	$(CXX) $(CXXFLAGS) -o $@ $(CLI_OBJ) $(LIB)

# 三路默认引脚：左/中在 gpiochip1，右在 gpiochip4
# 若板上编号不同，改 --chip1 / --chip4；只接了一路时用 --sensor 调试
run: $(TARGET)
	./$(TARGET) --chip1 1 --chip4 4 --raw

stats: $(TARGET)
	./$(TARGET) --stats 100

clean:
	rm -f $(TARGET) $(LIB) $(LIB_OBJ) $(CLI_OBJ)

distclean: clean
	rm -f calib.csv

.PHONY: all lib run stats clean distclean
