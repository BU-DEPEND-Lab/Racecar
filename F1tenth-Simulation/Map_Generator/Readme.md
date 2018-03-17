Build using 'javac Generator' to get Generator.class
run using 'java Generator'

## Randomly generates a map for Gazebo

#Procedure:

1. Put program parameters in "settings.txt"
2. Run Generator.jar
3. Look in /output/ for the Gazebo world (result.world), end location (coord.txt), and visualization map (map.png).

## Rules for settings.txt

Segment ->
1- Long-winding road 
	Feature1 -> Number of turns
	Feature2 -> complexity(0.05 - 0.95)
	
2- Grid-space
	Feature1 -> Number of Rows and columns
	Feature2 -> corrlidor length(0-1)
	Feature3 -> End point X coordinate in gridspace
	Feature4 -> End point Y coordinate in gridspace

3- Square Box
	Feature1 -> length(0-4)
	Feature2 -> height(0-4)
	Feature3 -> 0 - center
		    1 - top right
		    2 - bottom right
		    3 - bottom left

4- Triangle
	Feature1 -> angle1
	Feature2 -> angle2
	Feature3 -> constant (20+)

5- Circle
	Feature1 -> Radius
	Feature2 -> 0 - Origin at center
		    1 - Origin at perimeter

6- Curves
	Feature1 -> 0 - Sin (Phase Shifted by PI)
		    1 - 2 connected SemiCircles
	Feature2 -> Amplitute (for 0)
		    Radius 1 (for 1)
	Feature3 -> Frequency (for 0) 
		    Radius 2 (for 1)
