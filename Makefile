all : release

debug :
	mkdir -p bin-dbg
	cd bin-dbg && cmake -DCMAKE_BUILD_TYPE=Debug ../src && make

release :
	mkdir -p bin-opt
	cd bin-opt && cmake -DCMAKE_BUILD_TYPE=Release ../src && make -j8

clean :
	rm -rf bin-opt
	rm -rf bin-dbg

test : debug
	cd bin-dbg && ctest

test-release : release
	cd bin-opt && ctest

install : debug
	cd bin-dbg && make install

install-release : release
	cd bin-opt && sudo make install

uninstall : debug
	cd bin-dbg && make uninstall

uninstall-release : release
	cd bin-opt && make uninstall

documentation :
	cd doc && doxygen Doxyfile

kdevelop :
	mkdir -p kdevelop-pro
	cd kdevelop-pro && cmake -G KDevelop3 -DCMAKE_BUILD_TYPE=Debug ../src/

#TODO(pau) Add a constructor of CodeBlocks project using the wiki instructions.
