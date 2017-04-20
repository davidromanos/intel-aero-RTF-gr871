#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>


typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<float, 6, Dynamic> Matrix6kf;


/* ############################## Defines measurement class ##############################  */

class Measurement
{
public:
	/* variables */
	unsigned int c; 	/* measurement identifier - 0 for pose measurement and 1...N for landmark identifier */
	Eigen::VectorXf z	/* actual measurement - can take different sizes! */

	/* functions */
	Measurement();
	~Measurement();
	Eigen::Vector3f calculateHl();		/* calculates derivative of measurement model with respect to landmark variable - l */
	Vector6f calculateHs();		/* calculates derivative of measurement model with respect to pose variable - s */
private:
}

class LandmarkMeasurement : public Measurement
{
public:
	static Eigen::Matrix3f zCov 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/
	LandmarkMeasurement(unsigned int i);
private:
}

LandmarkMeasurement::LandmarkMeasurement(unsigned int i) 
{
 	c = i;
}

Vector6f LandmarkMeasurement::calculateHs(Vector6f pose,Eigen::Vector3f l) 
{
	s = pose; // temp variable to make it look like equation 	
	return Hs = s.topRows<3>()-l;
}

Eigen::Vector3f LandmarkMeasurement::calculateHl(unsigned int i) 
{
	s = pose; // temp variable to make it look like equation 	
	return Hs = s.bottomRows<3>()-l;
}

int ImageMeasurement::zCov << 1, 2, 3,
     			      4, 5, 6,
     			      7, 8, 9; // static variable - has to be declared outside class!



int main()
{
	LandmarkMeasurement z1(1);
	LandmarkMeasurement z2(2);
	LandmarkMeasurement z3(3);
	LandmarkMeasurement z4(4);

	
  	std::cout << "Hello World!" << std::endl;
	std::cout << z1.c << std::endl;
	std::cout << z3.c << std::endl;
  	return 0;
}

