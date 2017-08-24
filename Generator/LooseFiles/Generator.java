//for SDF version 1.4

import java.util.*;
import java.io.*;
import java.net.URISyntaxException;
import java.awt.image.*;
import javax.imageio.ImageIO;;

public class Generator1 {
	
	public static Scanner top;
	public static Scanner bot;
	public static Scanner box;
	public static PrintWriter pw;
	
	public static int boxn;
	
	public static boolean field[][];
	
	public static int xdir[];
	public static int ydir[];
	
	public static int maxx, maxy, minx, miny;
	
	public static int boxes[][];
	public static int resized[][];
	
	public static double size;
	
	public static String curdir;
	
	public static void main(String[] args) throws IOException, URISyntaxException {
		m:
		try{
		boxn = 0;
		
		curdir = Generator1.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();
		if(curdir.substring(curdir.length() - 3).equals("jar")) {
			curdir = curdir.substring(0, curdir.length()-13);
		}
		
		top = new Scanner(new File(curdir+"ref/topheader.txt"));
		bot = new Scanner(new File(curdir+"ref/bottomheader.txt"));
		pw = new PrintWriter("/home/f1/catkin_ws/src/racecar_simulator/racecar_gazebo/worlds/result.world");
		PrintWriter pws = new PrintWriter(curdir+"output/coord.txt");
		
		Scanner f = new Scanner(new File("settings.txt"));
		f.next();
		int segment = f.nextInt(); // read type of segment
		f.next();
		size = Double.parseDouble(f.next()); f.nextLine(); // read thickness of path
		f.next();
		int maxboxn = f.nextInt();  f.nextLine(); // read maximum limit of workable area

		/////////////////////////////generate a path ///////////////////////////
		xdir = new int[] {0, 1, 0, -1};
		ydir = new int[] {1, 0, -1, 0};
		int length = 0;
		boolean bad = true;
		int tries = 0;
		int xpos = 0, ypos = 0;
		field = new boolean[1000][1000]; // path matrix
		field[500][500] = true; // center point
			for(int i = -1; i <2; i++){
				for (int j = -1; j<2; j++){
					try{
						field[500+i][500+j] = true;
					}	
					catch(Exception e){
					}
				}
			}
			xpos = 500;
			ypos = 500;

		//////////////////////////// long-winding path /////////////////////////
		if(segment == 1){ 
			f.next();
			int maxturns = f.nextInt(); f.nextLine(); // number of turns in path
			f.next();
			double chance_thresh = Double.parseDouble(f.next()); f.nextLine(); // irregularity
			while(bad) { // while path not properly created
				tries++;
				
				minx = 500-1; maxx = 500+1; // start area
				miny = 500-1; maxy = 500+1;	// start area
				field = new boolean[1000][1000]; // path matrix
				field[500][500] = true; // center point
				int turns = 0;
				for(int i = -1; i <2; i++){
					for (int j = -1; j<2; j++){
						try{
							field[500+i][500+j] = true;
						}	
						catch(Exception e){
						}
					}
				}
				xpos = 500;
				ypos = 500;
				int dir = (int) (Math.random() * 4); // return number 0-4
				xpos = xpos + xdir[dir]; // move up/down/left/right
				ypos = ypos + ydir[dir]; // move up/down/left/right
				field[xpos][ypos] = true; // set path
				checkMinMax(xpos, ypos);
				length = 2;

				while(turns < maxturns) {
					bad = true;
					double chance = Math.random(); // to prevent straight line movement

					// if (outside the bounded box) or Adjacent point is filled or U[0,1] < 0.15 
					if(outOfBounds(xpos + xdir[dir], ypos + ydir[dir]) || checkAdjacency(xpos + xdir[dir], ypos + ydir[dir], dir) || chance < chance_thresh ) {
						
						boolean left, right;
						int tempdir;

						// check if right turn is viable
						tempdir = (dir + 1) % 4;
						right = !checkAdjacency(xpos + xdir[tempdir], ypos + ydir[tempdir], tempdir) 
								&& !outOfBounds(xpos + xdir[tempdir], ypos + ydir[tempdir]);
						
						// check if left turn is viable
						int tempdir0;
						tempdir0 = (dir - 1 + 4) % 4;
						left = !checkAdjacency(xpos + xdir[tempdir0], ypos + ydir[tempdir0], tempdir0)
								&& !outOfBounds(xpos + xdir[tempdir0], ypos + ydir[tempdir0]);
						
						if(left || right) {
							if(left && right) {
								if(Math.random() < .5) { // choose left or right
									dir = tempdir; 
								} else dir = tempdir0;
							} else if (right) 
								dir = tempdir;
							 else if (left) 
								dir = tempdir0;
							
							xpos = xpos + xdir[dir]; // move to next position
							ypos = ypos + ydir[dir]; // move to next position
							field[xpos][ypos] = true; // set path point
							checkMinMax(xpos, ypos);
							bad = false; // so far so good
							turns++; // taken a turn
						}
					} else { // go straight
						xpos = xpos + xdir[dir];
						ypos = ypos + ydir[dir];
						field[xpos][ypos] = true;
						checkMinMax(xpos, ypos);
						bad = false;
					}
					if(bad) {
						break;
					}
					length++;

				}
				if(length > maxboxn / 2 - 10) // too long
					bad = true;
				if(tries > 50000) { // failed too many times
					System.out.println("Tries limit exceeded, turn count too high");
					System.exit(0);
				}
			}
		}

		//////////////////////////////// Grid-space ////////////////////////////////
		else if(segment == 2){ 
			f.next();
			int grid_size = f.nextInt(); f.nextLine(); // number of rows and columns
			if(grid_size<2)
				grid_size = 2;
			f.next();
			double corridor = Double.parseDouble(f.next()); f.nextLine();	 // length of corridor
			int corridor_len =(int)( corridor * ((1495-1024)*(1/23.6212158203)*1000/2048));
			minx = 500-1; maxx = 500+(corridor_len*grid_size); // start area
			miny = 500-1; maxy = 500+(corridor_len*grid_size);	// start area
			

			for(int j = 0; j<=(grid_size-1)*corridor_len; j++){
				field[xpos+j][ypos] = true;
				if(j%corridor_len == 0){
					for(int k = 0; k<=(grid_size-1)*corridor_len; k++){
						field[xpos+j][ypos+k] = true;
					}
				}
			}
			for(int j = 0; j<=(grid_size-1)*corridor_len; j++){
				field[xpos][ypos+j] = true;
				if(j%corridor_len == 0){
					for(int k = 0; k<=(grid_size-1)*corridor_len; k++){
						field[xpos+k][ypos+j] = true;
					}
				}
			}
			f.next();
			xpos = (f.nextInt()*corridor_len)+500; f.nextLine(); // Goal X coordinate in gridspace
			f.next();
			ypos = (f.nextInt()*corridor_len)+500; f.nextLine(); // Goal Y coordinate in gridspace
		}

		//////////////////////////////// Square-box ////////////////////////////////
		else if(segment == 3){ 
			f.next();
			double box_len = Double.parseDouble(f.next()); f.nextLine(); // length
			int box_len_conv =(int)( box_len * ((1495-1024)*(1/23.6212158203)*1000/2048));
			f.next();
			double box_hght = Double.parseDouble(f.next()); f.nextLine(); // height
			int box_hght_conv =(int)( box_hght * ((1495-1024)*(1/23.6212158203)*1000/2048));
			minx = 500-1; maxx = 500+1; // start area
			miny = 500-1; maxy = 500+1;	// start area
			
			for(int j = 0; j<=box_hght_conv; j++){
					for(int k = 0; k<=box_len_conv; k++){
						field[xpos+j][ypos+k] = true;
						checkMinMax(xpos+j, ypos+k);
					}
				}

			f.next();
			int goal = f.nextInt(); f.nextLine(); // goal location
			if (goal == 0){
				xpos = (int)(box_hght_conv) + 500;
				ypos = (int)(box_len_conv) + 500;
			}
			else if (goal == 1){
				xpos = 500 +3;
				ypos = (int)(box_len_conv) + 500 - 3;
			}
			else if (goal == 2){
				xpos = (int)(box_hght_conv) + 500 - 3;
				ypos = (int)(box_len_conv) + 500 - 3;
			}
			else if (goal == 3){
				xpos = (int)(box_hght_conv) + 500 - 3;
				ypos = 500 + 3;
			}
		}
		/////////////////////////////////// Triangle ///////////////////////////////////
		else if(segment == 4){ 
			f.next();
			double ang1 = Math.toRadians(f.nextInt()); f.nextLine(); // Angle from origin
			f.next();
			double ang2 = Math.toRadians(f.nextInt()); f.nextLine(); // Opposite Angle
			f.next();
			int cons = f.nextInt(); f.nextLine(); // Constant of Equation
			minx = 500-cons-1; maxx = 500 + cons +1; // start area
			miny = 500-cons-1; maxy = 500 + cons +1;	// start area

			// Formulae for verticis
			double x1 = (cons/(Math.tan(ang1)+Math.tan(ang2)));
			double y1 = (cons*(Math.tan(ang1))/(Math.tan(ang1)+Math.tan(ang2)));
			double x2 = (cons/Math.tan(ang1));
			double y2 = 0;

			for(int i = -cons; i<cons ; i++){
				for(int j = -cons; j< cons ; j++){
					if((((int) (Math.tan(ang1)*i) == j) || 
						(((int) (Math.tan(ang2)*i)) == (-j + cons)) || 
						(j == 0)) && 
						((i>=0)&& (i <= x2) && 
						((j>=0) &&(j <= y1)))) {
						field[xpos+i][ypos+j] = true;
						for(int k = -1; k <2; k++){
							for (int l = -1; l<2; l++){
								try{
									field[xpos+i+k][ypos+j+l] = true;
								}	
								catch(Exception e){
								}
							}
						}
					} 
				}
			}
			// Formulae for Altitude
			xpos = (int)((x1+x2)/2)+500 + 2;
			ypos = (int)((y1+y2)/2)+500;
		}

		

		/////////////////////////////////// Circle ///////////////////////////////////
		else if(segment == 5){ 
			f.next();
			int rad = f.nextInt(); f.nextLine(); // Radius of the circle
			f.next();
			int origin = f.nextInt(); f.nextLine(); // starting point
			f.next();
			if (origin == 0){
				minx = 500-rad-1; maxx = 500 + rad +1; // start area
				miny = 500-rad-1; maxy = 500 + rad +1;	// start area
				for(int i = -rad; i<= rad; i++){
					for(int j = -rad; j<=rad; j++){
						if ((Math.abs(((i*i) + (j*j) - (rad*rad))) < 20) || ((j == 0) 
							&& (i >= 0))) {
							field[xpos+i][ypos+j] = true;
							for(int k = -1; k <2; k++){
								for (int l = -1; l<2; l++){
									try{
										field[xpos+i+k][ypos+j+l] = true;
									}	
									catch(Exception e){
									}
								}
							}
						}
					}
				}
				xpos = 500 - rad ;
				ypos = 500 ;
			}

			if (origin == 1){
				minx = 500-1; maxx = 500 + (2*rad) +1; // start area
				miny = 500-rad-1; maxy = 500 + rad +1;	// start area
				for(int i = -rad; i<= rad; i++){
					for(int j = -rad; j<=rad; j++){
						if ((Math.abs(((i*i) + (j*j) - (rad*rad))) < 20)) {
							field[xpos+i+rad][ypos+j] = true;
							for(int k = -1; k <2; k++){
								for (int l = -1; l<2; l++){
									try{
										field[xpos+i+k+rad][ypos+j+l] = true;
									}	
									catch(Exception e){
									}
								}
							}
						}
					}
				}
				xpos = 500 + (2*rad) ;
				ypos = 500 ;
			}
		}

		/////////////////////////////////// Curves ///////////////////////////////////
		else if(segment == 6){ 
			f.next();
			int curve = f.nextInt(); f.nextLine(); // Type of curve
			if(curve == 0){ //Sine
				f.next();
				int amp = f.nextInt(); f.nextLine(); // Amplitude
				f.next();
				double freq = Double.parseDouble(f.next()); f.nextLine(); // frequency
				minx = 500 - 2; maxx = 500 + (int)(2*Math.PI/freq) + 2; // start area
				miny = 500 - amp - 2; maxy = 500 + amp + 2;	// start area
				for(int i = 0; i <= (int)(2*Math.PI/freq) +1; i++){
					for(int j = - amp - 1; j <= amp + 1; j++){
						if (j == (int) (amp*Math.sin((freq*i) - Math.PI))) {
							field[xpos+i][ypos+j] = true;
							for(int k = -1; k <2; k++){
								for (int l = -1; l<2; l++){
									try{
										field[xpos+i+k][ypos+j+l] = true;
									}	
									catch(Exception e){
									}
								}
							}
						}
					}
				}
				xpos = 500 + (int)(2*Math.PI/freq);
				ypos = 500 ;
			}
			if(curve == 1){ //2 semicircles
				f.next();
				int r1 = f.nextInt(); f.nextLine(); // Radius 1
				f.next();
				int r2 = f.nextInt(); f.nextLine(); // Radius 2
				minx = 500-1; maxx = 500 + (2*r1) + (2*r2) +1; // start area
				miny = 500- r1 - r2-1; maxy = 500 + r1 +r2 +1;	// start area
				for(int i = -r1; i <= r1; i++){
					for(int j = 0; j <= r1; j++){
						if ((Math.abs(((i*i) + (j*j) - (r1*r1))) < 20)) {
							field[xpos+i+r1][ypos+j] = true;
							for(int k = -1; k <2; k++){
								for (int l = -1; l<2; l++){
									try{
										field[xpos+i+k+r1][ypos+j+l] = true;
									}	
									catch(Exception e){
									}
								}
							}
						}
					}
				}
				for(int i = -r2; i <= r2; i++){
					for(int j = -r2; j <= 0; j++){
						if ((Math.abs(((i*i) + (j*j) - (r1*r1))) < 20)) {
							field[xpos+i+(2*r1) +r2][ypos+j] = true;
							for(int k = -1; k <2; k++){
								for (int l = -1; l<2; l++){
									try{
										field[xpos+i+k+(2*r1)+r2][ypos+j+l] = true;
									}	
									catch(Exception e){
									}
								}
							}
						}
					}
				}



				
		}
	}
			

			



		
		
		// Print generated map
		System.out.println();
		System.out.print("minx = ");
		System.out.print(minx);
		System.out.print(" maxx = ");
		System.out.println(maxx);
		System.out.print("miny = ");
		System.out.print(miny);
		System.out.print(" maxy = ");
		System.out.println(maxy);
		for(int i = minx; i <= maxx; i++) {
			for(int j = miny; j <= maxy; j++) {
				if(i == 500 && j == 500)
					System.out.print("S");
				else if(i == xpos && j == ypos)
					System.out.print("G");
				else if(field[i][j])
					System.out.print("o");
				else System.out.print(".");
			} System.out.println();
		}
		System.out.println();
		
		//convert to 2048
		boolean[][] tfa = new boolean[2048][2048];
		for(int i = minx; i <= maxx; i++) 
			for(int j = miny; j <= maxy; j++) 
				if(field[i][j]) 
					for(int k = 0; k < size * 20; k++) 
						for(int l = 0; l < size*20; l++) 
							tfa[1024 + (i-500) * (int)(size * 20) + k][1024 +(j-500) * (int)(size * 20) + l] = true;
		System.out.println("converted to 2048");					
		// generate boxes array //////////////////////////////////////////////////////
		boxes = new int[maxx - minx + 3][maxy - miny + 3];
		fillBoxes(500, 500);
		System.out.println("fill boxes");					

		System.out.println(boxes.length);
		System.out.println(boxes[0].length);					

		findWalls(500 - minx + 1, 500 - miny + 1);
		System.out.println("findwalls");					

		System.out.println();
		System.out.println("path length: " + length);
		System.out.println("Generating world");
		genHeaders("top");
		//set up image
		BufferedImage img = new BufferedImage(2048, 2048, BufferedImage.TYPE_INT_RGB);

		//add state info //////////////////////////////////////////////////////
		for(int i = 0; i < boxes.length; i++) {
			for(int j = 0; j < boxes[0].length; j++) {
				if(boxes[i][j] == 3) {
					addBoxState((i-(500-minx-1)-2) *size + ((int)((Math.random()*0.2-1)*100)/100.0), (j-(500-miny-1)-2) * size + ((int)((Math.random()*0.2-1)*100)/100.0), 0.5f*size);
					
				}
			}
		}
		pw.println("    </state>");
		
		//add model info //////////////////////////////////////////////////////
		for(int i = 0; i < boxn; i++) {
			addBoxModel(i, size);
		}
		
		genHeaders("bot");
		
		System.out.println(boxn +" boxes generated");
		System.out.println();
		System.out.println("End point: " + (xpos-500) * size + ", " + (ypos - 500) * size);
		
		pw.close();
		System.out.println("World Generated");

		pws.println("start_point: 0, 0");
		pws.println("end_point(Gazebo): " + (xpos-500) * size + ", " + (ypos - 500) * size);
		pws.println("end_point(2048): " + ((xpos-500) * (int) (size * 20)) + ", " + ((ypos-500) * (int) (size * 20)));
		pws.close();
		
		for(int i = 0; i < 2048; i++) {
			for(int j = 0 ; j < 2048; j++) {
				if(tfa[i][j])
					img.setRGB(i, j, 16777215);
				else img.setRGB(i, j, 0);
			}
		}
		
		File imgfile = new File(curdir+"output/map.png");
		ImageIO.write(img, "png", imgfile);
		}
	catch (ArrayIndexOutOfBoundsException e)
	{
		break m;
	}

	}
	
	public static void genHeaders(String type) {
		boolean done = false;
		while(!done) {
			try {
				if(type.equals("top"))
					pw.println(top.nextLine());
				else if(type.equals("bot"))
					pw.println(bot.nextLine());
			} catch (Exception E) {
				done = true;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////
	// Name : addBoxState	                                                        //
	// Return type : void	                                                        //
	// Parameters : double xpos, double ypos, double zpos                           //
	// Function : Adds box state to the .world file (position)					 	//
	// Return : N/A  																//
	//////////////////////////////////////////////////////////////////////////////////	
	public static void addBoxState(double xpos, double ypos, double zpos) throws IOException, URISyntaxException {
		box = new Scanner(new File(curdir+"ref/box.txt"));
		for(int i = 1; i <= 9; i++) {
			String in = box.nextLine();
			if(i == 1) 
				pw.println("	    <model name='unit_box_"+boxn+"'>");
			else if(i == 4)
				pw.println("     		   <pose> "+xpos+" "+ypos+" "+zpos+" 0 -0 0</pose>");
			else
				pw.println(in);
		}
		boxn++;
	}
	
	//////////////////////////////////////////////////////////////////////////////////
	// Name : addBoxModel	                                                        //
	// Return type : void	                                                        //
	// Parameters : int n, double scale		                                        //
	// Function : Adds box model syntax to the .world file (size and scale)		 	//
	// Return : N/A  																//
	//////////////////////////////////////////////////////////////////////////////////
	public static void addBoxModel(int n, double scale) throws IOException, URISyntaxException {
		box = new Scanner(new File(curdir+"ref/box.txt"));
		for(int i = 1; i<= 9; i++) 
			box.nextLine();
		for(int i = 10; i <= 63; i++) {
			String in = box.nextLine();
			if(i == 10) 
				pw.println("    <model name='unit_box_"+n+"'>");
			else if(i == 27) {
				pw.println("              <size> "+scale+" "+scale+" "+scale+" </size>");
			} else if(i == 44) {
				pw.println("              <size> "+scale+" "+scale+" "+scale+" </size>");
			}
			else
				pw.println(in);
		}
	}
	
	//////////////////////////////////////////////////////////////////////////////////
	// Name : checkMinMax	                                                        //
	// Return type : void	                                                        //
	// Parameters : int x, int y 			                                        //
	// Function : Sets the working area of the path 							 	//
	// Return : N/A  																//
	//////////////////////////////////////////////////////////////////////////////////

	public static void checkMinMax(int x, int y) {
		if(y > maxy) maxy = y;
		if(y < miny) miny = y;
		if(x > maxx) maxx = x;
		if(x < minx) minx = x;
	}
	
	public static void fillBoxes(int x, int y) {
		//System.out.println("currently @" + (x-minx+1) + ", " + (y-miny+1));
		boxes[x-minx+1][y-miny+1] = 1;
		for(int i = 0; i < 4; i++) {
			if(field[x+xdir[i]][y+ydir[i]] && boxes[x-minx+1+xdir[i]][y-miny+1+ydir[i]] != 1)
				fillBoxes(x+xdir[i], y+ydir[i]);
		}
	}
	
	public static void findWalls(int x, int y) {
		boxes[x][y] = 2;
		for(int i = 0; i < 4; i++) {
			if(boxes[x+xdir[i]][y+ydir[i]] == 0)
				boxes[x+xdir[i]][y+ydir[i]] = 3;
			else if(boxes[x+xdir[i]][y+ydir[i]] == 1)
				findWalls(x+xdir[i], y+ydir[i]);
		}
	}
	

	//////////////////////////////////////////////////////////////////////////////////
	// Name : checkAdjacency                                                        //
	// Return type : boolean                                                        //
	// Parameters : int x, int y, int dir                                           //
	// Function : checks if there is an adjacent filled square at x, y location 	//
	// Return : TRUE if there is, FALSE if not.										//
	//////////////////////////////////////////////////////////////////////////////////

	public static boolean checkAdjacency(int x, int y, int dir) {
		for(int i = 0; i < 4; i++) {
			// if moving forward, dont check back. if moving left, dont check right, vice-versa.
			if(i != (dir + 2) % 4) 
				// if adjacent position filled
				if(field[x + xdir[i]][y+ydir[i]])
					return true;
		}
		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////
	// Name : outOfBounds                                                           //
	// Return type : boolean                                                        //
	// Parameters : int x, int y                                                    //
	// Function : checks if the path is being formed inside a bounded area 		    //
	// Return : TRUE if outside bounded box, FALSE if inside.						//
	//////////////////////////////////////////////////////////////////////////////////

	public static boolean outOfBounds(int x, int y) {
		double xpos = (x - 500) * size;
		double ypos = (y - 500) * size;
		if(xpos > 50 || xpos < -50 || ypos > 50 || ypos < -50) 
			return true;
		return false;
	}
}
