BUILD=build
UTEST=OFF
UDOC=ON
UEXAMPLES=OFF
CMAKE_ARGS:=$(CMAKE_ARGS)

all:
	@mkdir -p $(BUILD)
	@cd $(BUILD); cmake .. -DBUILD_TEST=$(UTEST) -DBUILD_DOC=$(UDOC) -DBUILD_EXAMPLES=$(UEXAMPLES) -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS) && $(MAKE)
	@echo -e "\n Now do 'make install' to install this package.\n"

apps:
	@$(MAKE) all UEXAMPLES=ON UTIMER=ON

unittest:
	@$(MAKE) all UTEST=ON
	@echo -e "\n\n Run test\n"
	@cd $(BUILD); $(MAKE) test

clean:
	@rm -rf $(BUILD)

install:
	@cd $(BUILD); $(MAKE) install
