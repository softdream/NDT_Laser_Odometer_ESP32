#ifndef __NDT_GRID_H
#define __NDT_GRID_H

#include <iostream>
#include <vector>
#include <limits>
#include <string.h>

#include "dataContainer.h"
#include "matrix_types.h"

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 1200

#define LASER_DIST 14.0
#define ROWS 28
#define COLUMNS 28
#define HALF_ROWS 14
#define HALF_COLUMNS 14

#define PI 3.14159265358979323846

namespace ndt
{

struct GridCell_
{
	
	GridCell_()
	{

	}
	
	~GridCell_()
	{

	}

	Point mean_;
  Covarince covarince_;
  int number_;

 	std::vector<Point> points;
  //Point points[400];
  //int points_number_;        
};
typedef struct GridCell_ GridCell;


class NdtGrid
{
public:
	NdtGrid() {  }
	~NdtGrid() {  } 

	bool ndtProcess( const slam::ScanContainer &first_scan,
			  const slam::ScanContainer &second_scan,
			  Pose &p,
			  const int max_iterations = 10 )
	{
		if( first_scan.isEmpty() || second_scan.isEmpty() ){
			return false;
		}	

		caculateNDTByFirstScan( first_scan );
		//std::cout<<"caculate the ndt ..."<<std::endl;
		int iteration = 0;
		for( ; iteration < max_iterations; iteration ++ ){
			
			
			estimateTransformationOnce( second_scan, p );
		}
    //std::cout<<"iterate the ndt ..."<<std::endl;
		
		angleNormalize( &p.theta );

		return true;
	}	

public:
	void caculateNDTByFirstScan( const slam::ScanContainer &scan )
	{
		//memset( grid.data(), 0, grid.size() * sizeof( grid[0] ) );
    for( int i = 0; i < grid.size(); i ++ ){
      grid[i].number_ = 0;
      memset( &grid[i].mean_, 0, sizeof( Point ) );
      memset( &grid[i].covarince_, 0, sizeof( Covarince ));
      grid[i].points.clear();
    }
    //std::cout<<"memset the grid ..."<<std::endl;

		for( size_t i = 0; i < scan.getSize(); i ++ ){
			Point point = scan.getIndexData( i );
			int index = pointMapToGrid( point );
			
		//	std::cout<<"--------------"<<std::endl;
		//	std::cout<<"point "<<i<<": "<<std::endl<<point.x<<std::endl<<point.y<<std::endl;
		//	std::cout<<"index : "<<index<<std::endl;
			grid[index].number_ ++;
			grid[index].mean_.x += point.x;
			grid[index].mean_.y += point.y;
			grid[index].points.push_back( point );
      //grid[index].points[grid[index].points_number_].x = point.x;
      //grid[index].points[grid[index].points_number_].y = point.y;
      //grid[index].points_number_ ++;
		}	
    //std::cout<<"caculate the number ..."<<std::endl;
    
		for( size_t i = 0; i < grid.size(); i ++ ){
			if( grid[i].number_ >= 3 ){
				Point average;
				average.x = grid[i].mean_.x / static_cast<float>( grid[i].number_ );
				average.y = grid[i].mean_.y / static_cast<float>( grid[i].number_ );

				grid[i].mean_.x = average.x;
				grid[i].mean_.y = average.y;
	
				for( int j = 0; j < grid[i].points.size(); j ++ ){
					float a = grid[i].points[j].x - grid[i].mean_.x;
          float b = grid[i].points[j].y - grid[i].mean_.y;
					grid[i].covarince_.a1 += a * a;
        	grid[i].covarince_.a2 += a * b;
        	grid[i].covarince_.a3 += b * a;
        	grid[i].covarince_.a4 += b * b;
				}
				
				grid[i].covarince_.a1 /= ( float )( grid[i].number_ - 1 );
      	grid[i].covarince_.a2 /= ( float )( grid[i].number_ - 1 );	
      	grid[i].covarince_.a3 /= ( float )( grid[i].number_ - 1 );
      	grid[i].covarince_.a4 /= ( float )( grid[i].number_ - 1 );
				//std::cout<<"covarince : "<< i<<std::endl<<grid[i].covarince_.a1<<" "<<grid[i].covarince_.a2<<std::endl<<grid[i].covarince_.a3<<" "<<grid[i].covarince_.a4<<std::endl;	
			}
			else {
				grid[i].covarince_.a1 = std::numeric_limits<float>::max();
                        	grid[i].covarince_.a2 = 0;
                        	grid[i].covarince_.a3 = 0;
                        	grid[i].covarince_.a4 = std::numeric_limits<float>::max();
			}
		}
		//std::cout<<"caculate the mean and covarince ..."<<std::endl;
	}

	void getHessianDerived( const slam::ScanContainer &scan, 
	    	           const Pose &p,
			   Hessian &H,
			   Dtr &b )
	{
		memset( &H, 0, sizeof( Hessian ) );
        	memset( &b, 0, sizeof( Dtr ) );
		
		for( size_t i = 0; i < scan.getSize(); i ++ ){
			// for all points in second scan frame, transform
                        Point point = scan.getIndexData( i );
			Point point_in_first_frame = pointCoordinateTransform( point, p );                
			
		        int index = pointMapToGrid( point_in_first_frame );
			//std::cout<<"--------------------------"<<std::endl;
			//std::cout<<"index : "<<index<<std::endl;			
			Point e;
			e.x = point_in_first_frame.x - grid[index].mean_.x;
			e.y = point_in_first_frame.y - grid[index].mean_.y;
			
			//std::cout<<"e : "<<std::endl<<e.x<<" "<<e.y<<std::endl;

			Covarince sigma_inverse;
			float det = grid[index].covarince_.a1 * grid[index].covarince_.a4 - grid[index].covarince_.a2 * grid[index].covarince_.a3;
			sigma_inverse.a1 = ( 1 / det ) * grid[index].covarince_.a4;
                	sigma_inverse.a2 = -( 1 / det ) * grid[index].covarince_.a2;
                	sigma_inverse.a3 = -( 1 / det ) * grid[index].covarince_.a3;
                	sigma_inverse.a4 = ( 1 / det ) * grid[index].covarince_.a1;
			
			//std::cout<<"sigma inverse : "<<i<<std::endl<<sigma_inverse.a1<< " "<<sigma_inverse.a2<<std::endl<<sigma_inverse.a3<<" "<<sigma_inverse.a4<<std::endl;

			float tmp1 = -point.x * sin( p.theta ) - point.y * cos( p.theta );
                	float tmp2 = point.x * cos( p.theta ) - point.y * sin( p.theta );
			//std::cout<<"tmp1 = "<<tmp1<<std::endl;
			//std::cout<<"tmp2 = "<<tmp2<<std::endl;
			b.a[0] += e.x * sigma_inverse.a1 + e.y * sigma_inverse.a3;
                	b.a[1] += e.x * sigma_inverse.a2 + e.y * sigma_inverse.a4;
                	b.a[2] += ( e.x * sigma_inverse.a1 + e.y * sigma_inverse.a3 ) * tmp1 + ( e.x * sigma_inverse.a2 + e.y * sigma_inverse.a4 ) * tmp2;

                	H.a[0][0] += sigma_inverse.a1;
                	H.a[0][1] += sigma_inverse.a2;
                	H.a[0][2] += sigma_inverse.a1 * tmp1 + sigma_inverse.a2 * tmp2;
                	H.a[1][0] += sigma_inverse.a3;
                	H.a[1][1] += sigma_inverse.a4;
                	H.a[1][2] += sigma_inverse.a3 * tmp1 + sigma_inverse.a4 * tmp2;
                	H.a[2][0] += sigma_inverse.a1 * tmp1 + sigma_inverse.a3 * tmp2;
                	H.a[2][1] += sigma_inverse.a2 * tmp1 + sigma_inverse.a4 * tmp2;
                	H.a[2][2] += ( sigma_inverse.a1 * tmp1 + sigma_inverse.a3 * tmp2 ) * tmp1 + ( sigma_inverse.a2 * tmp1 + sigma_inverse.a4 * tmp2 ) * tmp2;
		}
		//std::cout<<"b : "<<std::endl<<b.a[0]<<std::endl<<b.a[1]<<std::endl<<std::endl<<b.a[2]<<std::endl;
		//std::cout<<"H : "<<std::endl<<H.a[0][0]<<" "<<H.a[0][1]<<" "<<H.a[0][2]<<std::endl<<H.a[1][0]<<" "<<H.a[1][1]<<" "<<H.a[1][2]<<std::endl<<H.a[2][0]<<" "<<H.a[2][1]<<" "<<H.a[2][2]<<std::endl;
	}	

	void estimateTransformationOnce( const slam::ScanContainer &scan,
                           		 Pose &p )	
	{
		getHessianDerived( scan, p, H, b );

		float a1 = H.a[0][0];
	        float b1 = H.a[0][1];
        	float c1 = H.a[0][2];
	        float a2 = H.a[1][0];
        	float b2 = H.a[1][1];
	        float c2 = H.a[1][2];
        	float a3 = H.a[2][0];
	        float b3 = H.a[2][1];
        	float c3 = H.a[2][2];

        	float det = a1 * ( b2 * c3 - c2 * b3 ) - a2 * ( b1 * c3 - c1 * b3 ) + a3 * ( b1 * c2 - c1 * b2 );

	        Hessian H_inverse;
        	H_inverse.a[0][0] = ( 1 / det ) * ( b2 * c3 - c2 * b3 );
	        H_inverse.a[0][1] = ( 1 / det ) * ( c1 * b3 - b1 * c3 );
        	H_inverse.a[0][2] = ( 1 / det ) * ( b1 * c2 - c1 * b2 );
	        H_inverse.a[1][0] = ( 1 / det ) * ( c2 * a3 - a2 * c3 );
        	H_inverse.a[1][1] = ( 1 / det ) * ( a1 * c3 - c1 * a3 );
	        H_inverse.a[1][2] = ( 1 / det ) * ( a2 * c1 - a1 * c2 );
        	H_inverse.a[2][0] = ( 1 / det ) * ( a2 * b3 - b2 * a3 );
	        H_inverse.a[2][1] = ( 1 / det ) * ( b1 * a3 - a1 * b3 );
        	H_inverse.a[2][2] = ( 1 / det ) * ( a1 * b2 - a2 * b1 );

			//std::cout<<"H_inverse : "<<std::endl<<H_inverse.a[0][0]<<" "<<H_inverse.a[0][1]<<" "<<H_inverse.a[0][2]<<std::endl<<H_inverse.a[1][0]<<" "<<H_inverse.a[1][1]<<" "<<H_inverse.a[1][2]<<std::endl<<H_inverse.a[2][0]<<" "<<H_inverse.a[2][1]<<" "<<H_inverse.a[2][2]<<std::endl;

	        Pose delta_p;
        	delta_p.x = H_inverse.a[0][0] * b.a[0] + H_inverse.a[0][1] * b.a[1] + H_inverse.a[0][2] * b.a[2];
	        delta_p.y = H_inverse.a[1][0] * b.a[0] + H_inverse.a[1][1] * b.a[1] + H_inverse.a[1][2] * b.a[2];
        	delta_p.theta = H_inverse.a[2][0] * b.a[0] +  H_inverse.a[2][1] * b.a[1] +  H_inverse.a[2][2] * b.a[2];

	        p.x -= delta_p.x;
        	p.y -= delta_p.y;
	        p.theta -= delta_p.theta;
	}

	const int pointMapToGrid( const Point &point ) const
	{
		int x = point.x / (float)( LASER_DIST / HALF_COLUMNS );
        	int y = point.y / (float)( LASER_DIST / HALF_ROWS );
		//std::cout<<"x = "<<x<<", y = "<<y<<std::endl;		

		if( point.y >= 0 ){
			if( point.x >= 0 ){
				return ( HALF_ROWS - y - 1 ) * COLUMNS + ( x + HALF_COLUMNS + 1 );
			}
			else {
				return ( HALF_ROWS - y - 1 ) * COLUMNS + ( x + HALF_COLUMNS );
			}
		}
		else {
			if( point.x >= 0 ){
				return ( HALF_ROWS - y ) * COLUMNS + ( HALF_COLUMNS + x ) + 1;
			}
			else {
				return ( HALF_ROWS - y ) * COLUMNS + ( HALF_COLUMNS + x );
			}
		}
	}
	

	const Point pointCoordinateTransform( const Point &point_old, const Pose &delta ) const
	{
		float tmp_cos = cos( delta.theta );
        	float tmp_sin = sin( delta.theta );

	        Point ret;
        	ret.x = tmp_cos * point_old.x - tmp_sin * point_old.y + delta.x;
	        ret.y = tmp_sin * point_old.x + tmp_cos * point_old.y + delta.y;

        	return ret;
	}

	void angleNormalize( float *angle )
	{	
		if( *angle >= PI ){
			*angle -= 2 * PI;
		}

		if( *angle <= -PI ){
			*angle += 2 * PI;
		}
	}

private:

	std::vector<GridCell> grid = std::vector<GridCell>( COLUMNS * ROWS + 1 );
	Hessian H;
    Dtr b;
};

typedef NdtGrid NDT;


}

#endif
