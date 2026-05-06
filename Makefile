# Tildagon firmware build helpers
# Usage: make [build|clean|flash|monitor|all]

DOCKER_IMG := ghcr.io/emfcamp/esp_idf:v5.5.1
PORT       := /dev/ttyACM0
PWD_DIR    := $(shell pwd)
BUILD_DIR  := micropython/ports/esp32/build-tildagon

DOCKER_RUN := sudo docker run -it --rm \
              --env "TARGET=esp32s3" \
              -v "$(PWD_DIR)":/firmware \
              $(DOCKER_IMG)

DOCKER_RUN_TTY := sudo docker run -it --rm \
                  --device $(PORT):/dev/ttyUSB0 \
                  --env "TARGET=esp32s3" \
                  -v "$(PWD_DIR)":/firmware \
                  $(DOCKER_IMG)

.PHONY: build firstTime clean flash deploy monitor all help

help:
	@echo "Targets:"
	@echo "  build    - compile firmware"
	@echo "  clean    - delete build directory"
	@echo "  flash    - flash badge (alias: deploy). Badge must be in bootloader."
	@echo "  monitor  - attach serial monitor"
	@echo "  all      - clean + build + flash"
	@echo ""
	@echo "Bootloader: hold BOOP, plug USB, release after a few seconds."
	@echo "Port: $(PORT)  (override with: make flash PORT=/dev/ttyACMx)"


firstTime:
	./scripts/firstTime.sh

build: firstTime
	$(DOCKER_RUN)

clean:
	sudo rm -rf $(BUILD_DIR)

flash deploy:
	$(DOCKER_RUN_TTY) deploy

monitor:
	$(DOCKER_RUN_TTY) monitor

all: clean build flash
