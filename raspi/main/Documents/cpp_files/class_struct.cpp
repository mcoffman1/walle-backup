#include <iostream>
#include <string>
using namespace std;

struct robot_struct
{
	int id;
	int wheels;
	string name;
};

class robot_class
{
public:
	int id;
	int wheels;
	string name;
	void move();
	void stop();
};

class derived: public robot_class
{
public:
	void turnleft();
	void turnright();
};

void robot_class::move()
{
	cout<<"Moving Robot"<<endl;
}

void robot_class::stop()
{
	cout<<"Pooping"<<endl;
}

void derived::turnleft()
{
	cout<<"Turning Right"<<endl;
}

void derived::turnright()
{
	cout<<"Hanging a Lucy!!"<<endl;
}

int main()
{
	robot_struct first;
	robot_class second;
	derived third;
	
	first.id = 1;
	first.name = "Wall-E";
	
	second.id = 734;
	second.name = "Eve";
	
	third.id = 3;
	third.name = "RandoBot";
	
	cout<<"ID = "<<first.id<<"\t"<<"Name = "<<first.name<<endl;
	cout<<"ID = "<<second.id<<"\t"<<"Name = "<<second.name<<endl;
	cout<<"ID = "<<third.id<<"\t"<<"Name = "<<third.name<<endl;
	third.move();
	third.stop();
	third.turnleft();
	third.turnright();
	return 0;
}
