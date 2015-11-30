#ifndef LINALGEBRA_H_
#define LINALGEBRA_H_

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

template<typename T, size_t SIZE>
class Vector
{
	public:

		Vector()
		{
			for (int i=0; i<SIZE; ++i)
				this->data[i] = 0;
		}

		Vector (std::vector<T> raw_data)
		{
			assert( raw_data.size() == SIZE );
			for (int i=0; i<SIZE; ++i)
			{
				data[i] = raw_data[i];
			}
		}
		
		Vector<T,SIZE> operator+ (const Vector<T,SIZE>& v) const
		{
			Vector<T,SIZE> result;
			for (int i=0; i<SIZE; ++i)
				result.data[i] = this->data[i] + v.data[i];
			return result;
		}

		T operator* (const Vector<T,SIZE>& v) const
		{
			T result = 0;
			for (int i=0; i<SIZE; ++i) result += v.data[i]*this->data[i];
			return result;
		}

		Vector<T,SIZE> operator* (const T scalar) const
		{
			Vector<T,SIZE> result;
			for (int i=0; i<SIZE; ++i)
				result.data[i] = data[i]*scalar;
			return result;
		}

		friend Vector<T,SIZE> operator* (const T scalar, Vector<T,SIZE> v) 
		{
			Vector<T,SIZE> result;
			for (int i=0; i<SIZE; ++i)
				result.data[i] = v.data[i]*scalar;
			return result;
		}

		Vector<T,SIZE> operator- (const Vector<T,SIZE> v) const
		{
			Vector<T,SIZE> result;
			for (int i=0; i<SIZE; ++i)
				result.data[i] = this->data[i] - v.data[i];
			return result;
		}

		T& operator[] (const size_t i) { return data[i]; }

		friend std::ostream& operator<< (std::ostream& os, Vector<T,SIZE> v)
		{
			for (int i=0; i<SIZE; ++i)
			{
				os << v.data[i] << " ";
			}
			return os;
		}

		T norm()
		{
			T sum = 0;
			for (int i=0; i<SIZE; ++i) sum += data[i]*data[i];
			return sqrt(sum);
		}

		void normalize()
		{
			T inv_norm = 1/this->norm();
			for (int i=0; i<SIZE; ++i) data[i] *= inv_norm; 
		}

		T data[SIZE];
};

typedef Vector<float,2> Vector2f;
typedef Vector<float,3> Vector3f;
typedef Vector<float,4> Vector4f;

typedef Vector<double,2> Vector2d;
typedef Vector<double,3> Vector3d;
typedef Vector<double,4> Vector4d;

template<typename T>
Vector<T,3> cross( Vector<T,3> a, Vector<T,3> b)
{
	Vector<T,3> result;
	result[0] = a[1]*b[2] - a[2]*b[1];
	result[1] = a[2]*b[0] - a[0]*b[2];
	result[2] = a[0]*b[1] - a[1]*b[0];
	return result;
}

#endif
