#include "Radar.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include<iostream>
#include<cmath>


#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <string.h>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include<math.h>
#include<map>
#include "opencv2/core/core.hpp"

#define DATA_PORT "/dev/ttyACM1"
#define BAUD_RATE 921600


using namespace std;
using namespace cv;

void Radar::saveImage(){
	saveData = true;
}

void Radar::startRadar(){
	fd = open(DATA_PORT,  O_RDWR | O_NOCTTY | O_SYNC);
	if (fd == -1) /* Error Checking */
		printf("\n  Error! in Opening USB  ");
	else
		printf("\n Opened Successfully ");
	tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings, B921600); /* Set Read  Speed as 921600                     */
	cfsetospeed(&SerialPortSettings, B921600); /* Set Write Speed as 9600                       */

	//8N1 Mode
	SerialPortSettings.c_cflag &= ~PARENB;  //Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;  //CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;  //Clears the mask for setting the data size
	SerialPortSettings.c_cflag |= CS8;  //Set the data bits = 8

	SerialPortSettings.c_cflag &= ~CRTSCTS;//  No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;//  Enable receiver,Ignore Modem Control lines

	SerialPortSettings.c_iflag &= (IXON | IXOFF | IXANY);//  Disable XON/XOFF flow control both i/p and o/p
	SerialPortSettings.c_iflag &= (ICANON | ECHO | ECHOE | ISIG);//  Non Cannonical mode

	SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

	 //Setting Time outs
	SerialPortSettings.c_cc[VMIN] = 10;  //Read at least 10 characters
	SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly
	SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

	if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) { /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes");
	} else {
		printf("\n  BaudRate = 921600 \n  StopBits = 1 \n  Parity   = none");
	}
#if VISUAL
	//mkfifo(myfifo, 0666);
	//fdf = open(myfifo, O_WRONLY);
#endif
}

void Radar::closeRadar(){
	close(fd); /* Close the serial port */
#if VISUAL
/*	close(fdf);
	unlink(myfifo);*/
#endif
}

bool Radar::checkMagicWord(int readWord, int position){
	bool pass = (readWord == magicWordInts[position]);
	return pass;
}

void concatenateInfo(char& readInfo, int bytes_read){

}

//returns 1 if the data was read correctly, 0 otherwise
int Radar::readInfo(){
	//fd = open(DATA_PORT,  O_RDWR | O_NOCTTY | O_SYNC);
	tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */
	//Reading Header-------------------------------------------------------------
	char read_buffer[1824]; /* Buffer to store the data received              */
	memset(read_buffer,0,1824);
	size_t bytes_read= 0, bytes_expected = 1024;

	do {
	  ssize_t result = read(fd, read_buffer + bytes_read, bytes_expected - bytes_read);
	  if (0 >= result)
	  {
	    if (0 > result)
	    {
	      perror("read()");
	    }

	    break;
	  }
	  bool pass = true;
	  for(int i = 0; i <= 7;i++){
		 // cout << (int)read_buffer[i] << " ";
		  pass = pass && checkMagicWord(read_buffer[i],i);
	  }
	  //cout << endl;
	  if(pass){
		  unsigned int packetLength = 0;
		  	for (int i = 15; i >= 12; i--) {
		  		packetLength <<= 8;
		  		packetLength = packetLength + (int) read_buffer[i];
		  	}
		  bytes_read+= result; //should not increase until magic word
		  bytes_expected = packetLength;
	  }
	} while (bytes_read < bytes_expected);

	unsigned int detectedObjects = 0;
	for (int i = 31; i >= 28; i--) {
		detectedObjects <<= 8;
		detectedObjects = detectedObjects + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Number of Detected Objects: " << detectedObjects << endl;

	//printf("\n +----------------------------------+ \n\n");
	//Reading  Detected Objects ----------------------------------------------------------
	int sizeDetectedObjects = 12 + detectedObjects * 12;
	int byteToReadB = 36;

	unsigned int structureTag = 0;
	for (int i = byteToReadB + 3; i >= byteToReadB; i--) {
		structureTag <<= 8;
		structureTag = structureTag + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Structure Tag: " << structureTag << endl;
	byteToReadB = byteToReadB + 4;
	unsigned int lengthStruct = 0;
	for (int i = byteToReadB + 3; i >= byteToReadB; i--) {
		lengthStruct <<= 8;
		lengthStruct = lengthStruct + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Length Structure: " << lengthStruct << endl;
	byteToReadB = byteToReadB + 4;

	unsigned int Descriptor1 = 0;
	for (int i = byteToReadB + 1; i >= byteToReadB; i--) {
		Descriptor1 <<= 8;
		Descriptor1 = Descriptor1 + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	byteToReadB = byteToReadB + 2;
	//cout << "Descriptor Num of detected Objects: " << Descriptor1 << endl;
	unsigned int Descriptor2 = 0;
	for (int i = byteToReadB + 1; i >= byteToReadB; i--) {
		Descriptor2 <<= 8;
		Descriptor2 = Descriptor2 + (int) read_buffer[i];
		//cout << "Byte " << (unsigned int) read_buffer[i] << endl;
		//cout << (int) read_buffer[i] << " ";
	}
	//Descriptor2 = (unsigned int)read_bufferDO[11];
	//cout << "Descriptor 2:  " << Descriptor2 << endl;
	//cout << "Descriptor XYZ Q format: " << pow(2, Descriptor2) << endl;
	float xyzQ = (float) pow((float) 2, (float) (Descriptor2));
	byteToReadB = byteToReadB + 2;

	//data to be send ---------------
#if VISUAL
	stringstream stream;
	stream << detectedObjects;
	stream << "\n";
	if(saveData){
		stream << 1;
	}
	else{
		stream << 0;
	}
#endif
	dataPointNum = detectedObjects;
	//distanceToSB.clear();
	//xCoordinates.clear();
	fusion.clear();
	labels.clear();
	vector<PointD> DB;
	//cout << "Number Num: " << detectedObjects << " \n";
	for (int obj = 0; obj < detectedObjects; obj++) {
		union {
			char chars[2];
			int16_t in;
		} u1,u2;

		u1.chars[0] = read_buffer[byteToReadB + 6];
		u1.chars[1] = read_buffer[byteToReadB + 7];
		float xCoo = float(u1.in) / xyzQ;

		u2.chars[0] = read_buffer[byteToReadB + 8];
		u2.chars[1] = read_buffer[byteToReadB + 9];
		float yCoo = float(u2.in) / xyzQ;

		double mag = sqrt(xCoo * xCoo + yCoo * yCoo);
		double ang = atan2(yCoo, xCoo);
		if (xCoo < 0) {
			ang = M_PI - ang;
		}
		//distanceToSB.push_back(getDistancePointToStereo(mag, ang));
		//xCoordinates.push_back(xCoo);
		PointD p;
		p.x = xCoo;
		p.y = yCoo;
		DB.push_back(p);
		labels.insert(pair<PointD,int>(p,-1));

		/*stream << "\n";
		stream << fixed << setprecision(3) << xCoo;
		stream << " ";
		stream << fixed << setprecision(3) << yCoo;*/

		byteToReadB = byteToReadB + 12;
	}

	//DBSCAN
	int numberOfClusters = DBSCAN(DB,dbscanEPS,minClusterSize,labels);
	//cout << "Number of clusters: " << numberOfClusters << endl;
	vector<PointD> centroids = getCentroidsOfClusters(labels,numberOfClusters);
	//showClusterResults(labels);
#if VISUAL
	map<PointD, int>::iterator itr;
	for (itr = labels.begin(); itr != labels.end(); ++itr) {
		stream << "\n";
		stream << fixed << setprecision(3) << itr->first.x;
		stream << " ";
		stream << fixed << setprecision(3) << itr->first.y;
		stream << " ";
		stream << fixed << setprecision(3) << itr->second;
	}

	stream << "\n";
	mkfifo(myfifo, 0666);
	fdf = open(myfifo, O_WRONLY);
	//cout << stream.str();
	string SendInfo = stream.str();
	write(fdf, SendInfo.c_str(), SendInfo.length());
	close(fdf);
	unlink(myfifo);
#endif
	//cout << "done Passing info \n";
	return 1;
}

vector<PointD> Radar::getCentroidsOfClusters(map<PointD,int> labelsMap, int numberOfClusters){
	PointD initial;
	initial.x = 0;
	initial.y = 0;
	vector<PointD> centroids(numberOfClusters,initial); //One cluster number is noise (0)
	vector<int> pointsInCluster(numberOfClusters,0);
	map<PointD, int>::iterator itr;
	for (itr = labelsMap.begin(); itr != labelsMap.end(); ++itr) {
		if(itr->second > 0){ //Part of a cluster, not noise
			PointD current = centroids.at(itr->second -1);
			current.x = current.x + itr->first.x;
			current.y = current.y + itr->first.y;
			centroids.at(itr->second-1) = current;
			pointsInCluster.at(itr->second-1) =  pointsInCluster.at(itr->second-1) + 1;
		}
	}
	for(int i=0; i<centroids.size();i++){
		PointD cen = centroids.at(i);
		int number = pointsInCluster.at(i);
		cen.x = cen.x/number;
		cen.y = cen.y/number;
		centroids.at(i) = cen;
		double mag = sqrt(cen.x*cen.x + cen.y*cen.y);
		double ang = atan2(cen.y,cen.x);
		if(cen.x < 0){
			ang = M_PI - ang;
		}
		//distanceToSB.push_back(getDistancePointToStereo(mag, ang));
		//xCoordinates.push_back(cen.x);
		FusionInfo f;
		f.displacement = cen.x;
		f.distanceToSB = getDistancePointToStereo(mag, ang);
		fusion.push_back(f);

	}
	return centroids;
}

int Radar::DBSCAN(vector<PointD> DB, double eps, int minPts, map<PointD, int> &labels){
	int C = 0;
	for(PointD p : DB){
		if(labels.at(p) == -1){ //unvisited Nodes
			labels.at(p) = 0; //mark as visited
			vector<PointD> sphere_points = RangeQuery(DB,p,eps);
			if(sphere_points.size() >= minPts){
				C++;
				expandCluster(p,sphere_points,C,eps,minPts,labels,DB);
			}
		}
	}
	return C;
}

void Radar::expandCluster(PointD P, vector<PointD> sphere_points, int C, double eps, int minPts, map<PointD, int> &labels, vector<PointD> DB){
	labels.at(P) = C;
	int size = sphere_points.size();
	for(int i=0;i<size;i++){//for(PointD Pp : sphere_points){
		PointD Pp = sphere_points.at(i);
		if(labels.at(Pp) == -1){
			labels.at(Pp) = 0;
			vector<PointD> sphere_points_2 = RangeQuery(DB,Pp,eps);
			if(sphere_points_2.size() >= minPts){
				//sphere_points.insert(sphere_points.end(),sphere_points_2.begin(),sphere_points_2.end());
				expandVector(sphere_points,sphere_points_2);
				size = size + sphere_points_2.size();
			}
			labels.at(Pp) = C;
		}
	}
}

void Radar::expandVector(vector<PointD> &toExpand, vector<PointD> adder){
	for(PointD item : adder){
		toExpand.push_back(item);
	}
}

//Finds neighbour clusters
vector<PointD> Radar::RangeQuery(vector<PointD> DB, PointD Q,double eps){
	vector<PointD> Neighbours;
	for(PointD p : DB){
		double distance = EuclidianDistance(Q,p);
		if(distance <= eps){
			Neighbours.push_back(p);
		}
	}
	return Neighbours;
}

double Radar::EuclidianDistance(PointD a, PointD b){
	return sqrt(pow((a.x - b.x),2)+ pow((a.y-b.y),2));
}

void Radar::showClusterResults(map<PointD,int> labels){
	map<PointD, int>::iterator itr;
	cout << "\nClustering Result : \n";
	cout << "\tPointx\t y \tCluster\n";
	for (itr = labels.begin(); itr != labels.end(); ++itr) {
		cout << '\t' << itr->first.x<<" "<<itr->first.y << '\t' << itr->second << '\n';
	}
	cout << endl;
}

map<PointD, int> Radar::getLabelMap(){
	return labels;
}

double Radar::getDistancePointToStereo(double mag, double ang){
	//cout << mag << " " << ang << " ";
	double z = Rts + mag*sin(ang);
	return z;
}

void Radar::setData(double distanceToSBNum){
	Rts = distanceToSBNum;
}

int Radar::getDataPointNum(){
	return dataPointNum;
}
