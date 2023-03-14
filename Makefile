ifdef OS
	gradleCmd = gradlew
else
	gradleCmd = ./gradlew
endif


all: build

movePaths:
	mv ./src/main/deploy/pathplanner/generatedJSON/New\ Path.waypoints.wpilib.json ./src/main/deploy/paths
	mv ./src/main/deploy/paths/New\ Path.waypoints.wpilib.json ./src/main/deploy/paths/Path.json

submod:
	python ./scripts/main.py

build: submod
	$(gradleCmd) build
