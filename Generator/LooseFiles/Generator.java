//for SDF version 1.4

import java.util.*;
import java.io.*;
import java.net.URISyntaxException;
import java.awt.image.*;
import javax.imageio.ImageIO;;

public class Generator {
	
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
		
		boxn = 0;
		
		curdir = Generator.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();
		if(curdir.substring(curdir.length() - 3).equals("jar")) {
			curdir = curdir.substring(0, curdir.length()-13);
		}
		
		top = new Scanner(new File(curdir+"ref/topheader.txt"));
		bot = new Scanner(new File(curdir+"ref/bottomheader.txt"));
		pw = new PrintWriter(curdir+"output/result.world");
		PrintWriter pws = new PrintWriter(curdir+"output/coord.txt");
		
		Scanner f = new Scanner(new File("settings.txt"));
		//System.out.print("# of turns: ");
		f.next();
		int maxturns = f.nextInt(); f.nextLine();
		
		//System.out.print("path size: ");
		f.next();
		size = Double.parseDouble(f.next()); f.nextLine();
		
		//System.out.print("max boxes: ");
		f.next();
		int maxboxn = f.nextInt(); // f.nextLine();
		
		//generate a path //////////////////////////////////////////////////////
		xdir = new int[] {0, 1, 0, -1};
		ydir = new int[] {1, 0, -1, 0};
		int length = 0;
		boolean bad = true;
		int tries = 0;
		
		int xpos = 0, ypos = 0;
		while(bad) {
			tries++;
			field = new boolean[1000][1000];
			
			minx = 10001; maxx = -1;
			miny = 10001; maxy = -1;
			
			int turns = 0;
			field[500][500] = true;
			xpos = 500;
			ypos = 500;
			checkMinMax(xpos, ypos);
			int dir = (int) (Math.random() * 4);
			xpos = xpos + xdir[dir];
			ypos = ypos + ydir[dir];
			field[xpos][ypos] = true;
			checkMinMax(xpos, ypos);
			
			length = 2;
			while(turns < maxturns) {
				bad = true;
				double chance = Math.random();
				//System.out.println(chance);
				if(outOfBounds(xpos + xdir[dir], ypos + ydir[dir]) || checkAdjacency(xpos + xdir[dir], ypos + ydir[dir], dir) || chance < .33 ) {
					boolean left, right;
					
					int tempdir = dir;
					tempdir = (dir + 1) % 4;
					
					right = !checkAdjacency(xpos + xdir[tempdir], ypos + ydir[tempdir], tempdir) 
							&& !outOfBounds(xpos + xdir[tempdir], ypos + ydir[tempdir]);
					
					int tempdir0 = dir;
					tempdir0 = (dir - 1 + 4) % 4;
					left = !checkAdjacency(xpos + xdir[tempdir0], ypos + ydir[tempdir0], tempdir0)
							&& !outOfBounds(xpos + xdir[tempdir0], ypos + ydir[tempdir0]);
					if(left || right) {
						if(left && right) {
							if(Math.random() < .5) {
								dir = tempdir;
							} else dir = tempdir0;
						} else if (right) 
							dir = tempdir;
						 else if (left) 
							dir = tempdir0;
						
						/*
						 * add dead ends
						chance = Math.random();
						if(chance < .4 && !checkAdjacency(xpos - xdir[dir], ypos - ydir[dir], (dir+2)%4)) {
							field[xpos - xdir[dir]][ypos - ydir[dir]] = true;
							checkMinMax(xpos - xdir[dir], ypos - ydir[dir]);
						}
						*/
						xpos = xpos + xdir[dir];
						ypos = ypos + ydir[dir];
						field[xpos][ypos] = true;
						checkMinMax(xpos, ypos);
						bad = false;
						turns++;
					}
				} else {
					xpos = xpos + xdir[dir];
					ypos = ypos + ydir[dir];
					field[xpos][ypos] = true;
					checkMinMax(xpos, ypos);
					bad = false;
				}
				//System.out.println(xpos + ", " + ypos);
				if(bad) {
					//System.out.println("error generating map, retrying");
					break;
				}
				length++;
				/*
				for(int i = minx; i <= maxx; i++) {
					for(int j = miny; j <= maxy; j++) {
						if(field[i][j])
							System.out.print("o");
						else System.out.print(".");
					} System.out.println();
				}
				System.out.println(); */
			}
			if(length > maxboxn / 2 - 10) 
				bad = true;
			if(tries > 50000) {
				System.out.println("Tries limit exceeded, turn count too high");
				System.exit(0);
			}
		}
		
		for(int i = minx; i <= maxx; i++) {
			for(int j = miny; j <= maxy; j++) {
				if(i == 500 && j == 500)
					System.out.print("S");
				else if(field[i][j])
					System.out.print("o");
				else System.out.print(".");
			} System.out.println();
		}
		for(int i = 0; i <= maxy - miny; i++) {
			if(i % 10 == 0)
				System.out.print(".");
			else System.out.print(i % 10);
		} System.out.println();
		
		//convert to 2048
		boolean[][] tfa = new boolean[2048][2048];
		for(int i = minx; i <= maxx; i++) 
			for(int j = miny; j <= maxy; j++) 
				if(field[i][j]) 
					for(int k = 0; k < size * 20; k++) 
						for(int l = 0; l < size*20; l++) 
							tfa[1024 + (i-500) * (int)(size * 20) + k][1024 +(j-500) * (int)(size * 20) + l] = true;
							
		// generate boxes array //////////////////////////////////////////////////////
		boxes = new int[maxx - minx + 3][maxy - miny + 3];
		fillBoxes(500, 500);
		/*
		for(int i = 0; i < boxes.length; i++) {
			for(int j = 0; j < boxes[i].length; j++) {
				if(boxes[i][j] != 0)
					System.out.print(boxes[i][j]);
				else System.out.print(".");
			}
			System.out.println();
		}
		*/
		findWalls(500 - minx + 1, 500 - miny + 1);
		
		System.out.println();
		System.out.println("generated in " + tries + " tries");
		System.out.println("path length: " + length);
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
	
	//returns true if there is an adjacent filled square at x, y location
	public static boolean checkAdjacency(int x, int y, int dir) {
		for(int i = 0; i < 4; i++) {
			if(i != (dir + 2) % 4) 
				if(field[x + xdir[i]][y+ydir[i]])
					return true;
		}
		return false;
	}
	
	public static boolean outOfBounds(int x, int y) {
		double xpos = (x - 500) * size;
		double ypos = (y - 500) * size;
		if(xpos > 50 || xpos < -50 || ypos > 50 || ypos < -50) 
			return true;
		return false;
	}
}
