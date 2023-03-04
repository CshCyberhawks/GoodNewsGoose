all: build

submod:
	python ./scripts/main.py

build: submod
	./gradlew build
