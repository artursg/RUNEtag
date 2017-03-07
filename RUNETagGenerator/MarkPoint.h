#ifndef MARK_POINT_H
#define MARK_POINT_H


#include <vector>
#include <complex>

template < typename T >
class MarkPoint {
public:
	std::complex<T> coords;
	T angle;
	T size;
	unsigned int layer;
	bool enabled;

	MarkPoint( T x, T y, T _angle, T _size, unsigned int _layer ) : coords( x,y ), angle(_angle), size(_size), layer(_layer), enabled(true) {};
	virtual ~MarkPoint(void) {};

	
	static double sin_angle( const MarkPoint& A, const MarkPoint& O, const MarkPoint& B ) {
		T v1_x = A.coords.real() - O.coords.real();//a1
		T v1_y = A.coords.imag() - O.coords.imag();//a2

		T v2_x = B.coords.real() - O.coords.real();//b1
		T v2_y = B.coords.imag() - O.coords.imag();//b2

		T cross = v1_x*v2_y - v1_y*v2_x;
		T v1_mag = sqrt( v1_x*v1_x + v1_y*v1_y);
		T v2_mag = sqrt( v2_x*v2_x + v2_y*v2_y);

		return cross/(v1_mag*v2_mag);
	}

	///<returns>Euclidean distance between this and another MarkPoint</returns>
	T distanceTo( const MarkPoint& other ) const {
		return sqrt(  (coords.real()-other.coords.real())*(coords.real()-other.coords.real()) + (coords.imag()-other.coords.imag())*(coords.imag()-other.coords.imag()) );
	}

	///<returns>True if the angle of this MarkPoint is less than the angle of other MarkPoint from the origin, false otherwise</returns>
	bool operator<( const MarkPoint& other ) const {
		if( angle < other.angle ) {
			return true;
		}
		if( angle > other.angle ) {
			return false;
		}
		return (  layer < other.layer ) ;
	}
};


typedef MarkPoint<double> MarkPointd;


#endif