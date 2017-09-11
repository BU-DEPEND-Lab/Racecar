#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>

using namespace std;

int main () {
  vector<double> x, y, speed;
  string line;
	const char *tmp;
  ifstream myfile("example.txt");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
	if(line == "")
		break;
	
	tmp = line.c_str();
      x.push_back(strtod(tmp,NULL));
	getline (myfile,line);
      	tmp = line.c_str();
      y.push_back(strtod(tmp,NULL));
	getline (myfile,line);
	tmp = line.c_str();
	speed.push_back(strtod(tmp,NULL));
cout << speed.back() << endl;
    }
    myfile.close();
  }

  else cout << "Unable to open file"; 

  return 0;
}
