ifdef OS
	gradleCmd = gradlew
else
	gradleCmd = ./gradlew
endif


all: build

submod:
	python ./scripts/main.py

build: submod
	$(gradleCmd) build
