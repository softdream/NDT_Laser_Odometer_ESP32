#ifndef __DATA_CONTAINER_H_
#define __DATA_CONTAINER_H_

#include <vector>
#include <cmath>

#include "matrix_types.h"

namespace slam {

class DataContainer
{
public:
	DataContainer() {  }
	~DataContainer() {  }

	void addData( const Point &data )
	{
		dataVec.push_back( data );		
	}

	void clear()
	{
		return dataVec.clear();
	}
	
	const Point& getIndexData( int index ) const
	{
		return dataVec[index];
	}

	const int getSize() const
	{
		return dataVec.size();
	}

	bool isEmpty() const
	{
		return dataVec.empty();
	}
private:
	std::vector<Point> dataVec;
	
};

typedef DataContainer ScanContainer;

}

#endif
