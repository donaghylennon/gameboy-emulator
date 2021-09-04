
gbemu: CPU.cpp CPU.h Memory.cpp Memory.h PPU.cpp PPU.h main.cpp
	g++ -lSDL2 main.cpp CPU.cpp Memory.cpp PPU.cpp -o gbemu
