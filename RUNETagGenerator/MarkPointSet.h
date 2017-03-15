#ifndef MARK_POINT_SET_H
#define MARK_POINT_SET_H

#include <vector>
#include <limits>
#include <complex>
#include <algorithm>
#include "MarkPoint.h"



/*
inline double cr( double z1, double z2, double z3, double z4 ) {
    return ( ((z1-z3)*(z2-z4))/( (z2-z3)*(z1-z4) ) );
	//return ( ((z1-z2)*(z3-z4))/( (z3-z2)*(z1-z4) ) );
}
*/


template <typename T>
std::complex<T> cr( std::complex<T> z1, std::complex<T> z2, std::complex<T> z3, std::complex<T> z4 ) {
    return ( ((z1-z3)*(z2-z4))/( (z1-z2)*(z3-z4) ) );
}

template <typename T>
std::complex<T> cr_ang( T sin12, T sin24, T sin13, T sin34 ) {
    return ( (sin12/sin24) / (sin13/sin34) );
}


template <typename T>
class MarkPointQuadruple;


template <typename T>
class MarkPointSet
{
public:
    MarkPointSet() {}
	virtual ~MarkPointSet(void) { points.clear(); }

	std::complex<T> crossRatioComplex( const MarkPointQuadruple<T>& quad, int p_index ) const {
		const MarkPoint<T>& origin = points.at( p_index );
		if( &quad.p1 == &origin || &quad.p2 == &origin || &quad.p3 == &origin || &quad.p4 == &origin )
			return std::complex<T>(0.0,0.0);
		
		std::complex<T> p1=quad.p1.coords;
		std::complex<T> p2=quad.p2.coords;
		std::complex<T> p3=quad.p3.coords;
		std::complex<T> p4=quad.p4.coords;

		return cr( p1, p2, p3, p4);
	}

	std::complex<T> crossRatioAngle( const MarkPointQuadruple<T>& quad, int p_index ) const {
		const MarkPoint<T>& origin = points.at( p_index );
		if( &quad.p1 == &origin || &quad.p2 == &origin || &quad.p3 == &origin || &quad.p4 == &origin )
			return std::complex<T>(0.0,0.0);
		
		T sin12 = MarkPoint<T>::sin_angle( quad.p1, origin, quad.p2 );
		T sin24 = MarkPoint<T>::sin_angle( quad.p2, origin, quad.p4 );
		T sin13 = MarkPoint<T>::sin_angle( quad.p1, origin, quad.p3 );
		T sin34 = MarkPoint<T>::sin_angle( quad.p3, origin, quad.p4 );

		return cr_ang( sin12, sin24, sin13, sin34 );
	}

	MarkPoint<T>& pointAt( unsigned int i ) {
		return points.at(i);
	}

	MarkPointQuadruple<T> randomQuadruple() const  {
		return MarkPointQuadruple<T>( points[0],points[1],points[2],points[3] );
	}

	MarkPointQuadruple<T> quadruple( int start_index ) const  {
		return MarkPointQuadruple<T>( points[start_index],
									  points[(start_index+1)%numPoints()],
									  points[(start_index+2)%numPoints()],
									  points[(start_index+3)%numPoints()] );
	}

	void addPoint( const MarkPoint<T>& p ) {
		points.push_back( p );
	}

	bool addPointDistanceThreshold( const MarkPoint<T>& p, T size )
    {
        typename std::vector< MarkPoint<T> >::const_iterator it = begin();

        bool valid = true;
        while( it != end() ) {
	  if( it->distanceTo( p ) < 1 /*threshold*/ ) {
                valid = false;
                break;
            }
            it++;
        }
        if( valid ) 
            addPoint( p );

        return valid;
    }
	
	unsigned int numPoints() const {
		return points.size();
	}

	void clear() {
		points.clear();
	}

	void sort() {
		std::sort( points.begin(), points.end() );
	}

	typename std::vector< MarkPoint<T> >::const_iterator begin() const {
		return points.begin();
	}


	typename std::vector< MarkPoint<T> >::const_iterator end() const {
		return points.end();
	}


	MarkPoint<T> nearest_to( const MarkPoint<T>& other, double* out_dist = 0 ) const
    {
        typename std::vector< MarkPoint<T> >::const_iterator it = begin();

        MarkPoint<T>& nearest = const_cast< MarkPoint<T>& >( *it );
        T min_dist = other.distanceTo( *it );

        it++;
        while( it!=end() ) {
            T distance = other.distanceTo( *it );

            if( distance < min_dist ) {
                min_dist = distance;
                nearest = *it;
            }
            it++;
        }

        if( out_dist ) {
            *out_dist = min_dist;
        }

        return nearest;
    }

private:
	std::vector< MarkPoint<T> > points;

};

template <typename T>
class MarkPointQuadruple 
{
friend class MarkPointSet<T>;
public:
	const MarkPoint<T>& p1;
	const MarkPoint<T>& p2;
	const MarkPoint<T>& p3;
	const MarkPoint<T>& p4;
private:
	MarkPointQuadruple( const MarkPoint<T>& _p1, const MarkPoint<T>& _p2, const MarkPoint<T>& _p3, const MarkPoint<T>& _p4  ) : p1(_p1), p2(_p2), p3(_p3), p4(_p4) {}
};



#include "MarkPointSet.cpp"

typedef MarkPointSet<double> MarkPointSetd;

#endif
