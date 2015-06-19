#include <iostream>

#include <math.h>

using std::cout;
using std::endl;

long counter{};
long millis(){
	cout << "millis: " << (counter+=10) << endl;
	return counter;
}
class Servo{
public:
	int p;
	void attach(int p){ this->p=p; cout << "attach on pin " << p << endl;}
	int ms;
	void writeMicroseconds(int ms){ this->ms=ms; cout << p << ".writeMicroseconds:" << ms << endl;}


};

#include "walker.h"

int main(){

	Walker w(9, 10); 

	w.move(10);

	while(w.update());

	w.rotate(M_PI/4);

	while(w.update());

	w.rotate(-M_PI/4);

	while(w.update());

}