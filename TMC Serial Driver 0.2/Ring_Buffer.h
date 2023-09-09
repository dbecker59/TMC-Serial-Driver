#ifndef WRAP_h
#define WRAP_h
#include <algorithm>

#define WRAP_DEFAULT_CAPACITY 64u					// Default container capacity
#define WRAP_RESIZE_WEIGHT 0.2f						// Default multiplier used for reallocation, reallocate(capacity * WRAP_RESIZE_WEIGHT);
#define WRAP_NORMALIZE_REALLOCATE_BIAS 0.8f			// Default multiplier used in normalization, if size > WRAP_NORMALIZ_REALLOCATE_BIAS * capacity, then reallocate


template<typename _T>
class Ring_Buffer
{
	_T* _front;										// Pointer to the first element of the allocated memory space
	_T* _back;										// Pointer to just past the last element of the allocated memory space
	_T* _first;										// Pointer to the first element in memory
	_T* _last;										// Pointer to just past the last element in memory

	// reallocates the memory to a new larger block
	void reallocate();

	// reallocates the memory to a new larger block
	//		_numElem:
	//			Number of elements to increase the size by
	void reallocate(const size_t _numElem);

public:
	// Ring_Buffer<_T>'s iterator
	class iterator;

	// returns the iterator of the first element in the container
	iterator begin() const;

	// returns the iterator just past the last element in the container
	iterator end() const;

	// default constructor
	Ring_Buffer();

	// Non-Default constructor
	//		reserve:
	//			Size (in _T elements) to initialize the container as.
	Ring_Buffer(const size_t reserve);

	// Default destructor
	~Ring_Buffer();

	// Returns the number of elements in the container
	size_t size() const;

	// Returns the number of elements the container can hold before reallocation
	size_t capacity() const;

	// Returns whether the container is empty
	bool empty() const volatile;

	// Normalizes the container
	//		- Shifts all elements to the front of the memory block
	//		- Reallocates the memory block to a larger block if the container is near capacity
	bool normalize();
	
	// Appends the element to the end of the container
	//		_elem:
	//			Element to be appended
	bool push(const _T& _elem);

	// Appends an array of elements to the end of the container
	//		buffer:
	//			Pointer to the array of elements to be appended
	//		_numElem:
	//			Number of elements in 'buffer' to be appended
	//		NOTE:	_numElem must not be more than the size of buffer!
	//				i.e. 'buffer' to 'buffer + _numElem' must be initiallized memory!
	bool push(const _T* const buffer, const size_t _numElem);

	// Removes the first element in the container
	void pop();

	// Removes a number of elements from the front of the container
	//		_numElem:
	//			Number of elements to remove from the front of the container
	void pop(const size_t _numElem);

	// Removes all elements from the front of the container up to and including the first instance of the terminator
	//		term:
	//			The terminator of the pop operation, all elements up to and including the first instance of term are removed
	//		NOTE: if there is no element equal to the 'term' in the container, no elements are removed
	void pop(const _T& term);

	// Clears all elements in the container
	void clear();

	// Copies the first element of the container
	//		popOff:
	//			Determines whether to leave the copied element or whether to remove it.
	//		NOTE: returns the default element '_T()' if container is empty, consider testing if empty before use.
	_T pull(bool popOff = true);

	// Copies the first '_numElem' elements of the container
	//		_numElem:
	//			The number of elements to copy from the front of the container
	//		popOff:
	//			Determines whether to leave the copied elements or whether to remove them.
	//		NOTE: if the number of elements requested is larger than the number of elements in the container
	//					then the function returns nullptr, consider checking the return value.
	_T* pull(const size_t _numElem, bool popOff = true);

	// Copies the elements in the container up to and including the terminator
	//		term:
	//			The terminator, the last element to be copied
	//		popOff:
	//			Determines whether to leave the copied elements or whether to remove them.
	//		NOTE: if there is no element equal to 'term' in the container, then nullptr is returned;
	//					consider checking the return value.
	_T* pull(const _T& term, bool popOff = true);


	// Returns an iterator to the first instance of 'elem' in the container.
	//		elem:
	//			Value to search for in the container
	//		NOTE: if there is no element in the container equal to 'elem', the function returns
	//					an iterator just past the end of the container (equal to Serial::end())
	iterator find(const _T& elem);
};


// Ring_Buffer<_T>'s iterator
template<typename _T>
class Ring_Buffer<_T>::iterator {
	const _T* _ptr;

public:
	iterator();
	iterator(const _T* ptr);
	iterator(const iterator& itr);

	~iterator();

	operator const _T*() const;

	iterator& operator=(const iterator& itr);

	const _T& operator*() const;
	iterator operator+(const int& rhs) const;
	iterator operator-(const int& rhs) const;
	int operator-(const iterator& rhs) const;

	iterator&  operator++();
	iterator operator++(int);
	iterator&  operator--();
	iterator operator--(int);

	bool operator<(const iterator& rhs) const;
	bool operator>(const iterator& rhs) const;
	bool operator==(const iterator& rhs) const;
	bool operator<=(const iterator& rhs) const;
	bool operator>=(const iterator& rhs) const;
};


// reallocates the memory to a new larger block
template<typename _T>
inline void Ring_Buffer<_T>::reallocate()
{
	reallocate(WRAP_RESIZE_WEIGHT * capacity());
}


// reallocates the memory to a new larger block
//		_numElem:
//			Number of elements to increase the size by
template<typename _T>
void Ring_Buffer<_T>::reallocate(const size_t _numElem)
{
	_T* newFront = new _T[capacity() + _numElem];
	memmove(newFront, _first, sizeof(_T) * size());
	delete[] _front;

	_back = newFront + capacity() + _numElem;
	_front = newFront;
	_last = _front + size();
	_first = _front;

#ifdef _DEBUG	// if in debug, set memory to 0
	memset(_last, 0, sizeof(_T) * (capacity() - size()));
#endif
}


// returns the iterator of the first element in the container
template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::begin() const
{
	return iterator(_first);
}


// returns the iterator just past the last element in the container
template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::end() const
{
	return iterator(_last);
}


// default constructor
template<typename _T>
Ring_Buffer<_T>::Ring_Buffer() :
	_front(new _T[WRAP_DEFAULT_CAPACITY]),
	_back(_front + WRAP_DEFAULT_CAPACITY),
	_first(_front),
	_last(_front)
{}


// Non-Default constructor
//		reserve:
//			Size (in elements) to initialize the container as.
template<typename _T>
Ring_Buffer<_T>::Ring_Buffer(const size_t reserve) :
	_front(new _T[reserve]),
	_back(_front + reserve),
	_first(_front),
	_last(_front)
{}


// Default destructor
template<typename _T>
Ring_Buffer<_T>::~Ring_Buffer()
{
	delete[] _front;
}


// Returns the number of elements in the container
template<typename _T>
inline size_t Ring_Buffer<_T>::size() const
{
	return _last - _first;
}


// Returns the number of elements the container can hold before reallocation
template<typename _T>
inline size_t Ring_Buffer<_T>::capacity() const
{
	return _back - _front;
}

// Returns whether the container is empty
template<typename _T>
inline bool Ring_Buffer<_T>::empty() const volatile
{
	return !(_first < _last);
}


// Normalizes the container
//		- Shifts all elements to the front of the memory block
//		- Reallocates the memory block to a larger block if the container is near capacity
template<typename _T>
inline bool Ring_Buffer<_T>::normalize()
{
	if (!(size() < (WRAP_NORMALIZE_REALLOCATE_BIAS * capacity())))
	{
		reallocate();
		return true;
	}
	if (_first == _front)
	{
		return false;
	}
	memmove(_front, _first, sizeof(_T) * size());
	_last = _front + size();
	_first = _front;

	#ifdef _DEBUG	// if in debug, set memory to 0
	memset(_last, 0, sizeof(_T) * (_back - _last));
	#endif
	return true;
}


// Appends the element to the end of the container
//		_elem:
//			Element to be appended
template<typename _T>
bool Ring_Buffer<_T>::push(const _T& _elem)
{
	if (_last == _back) {
		if (_first == _front)
		{
			reallocate();
		}
		else {
			memmove(_front, _first, sizeof(_T) * (_last - _first));
			_last = _front + size();
			_first = _front;

#ifdef _DEBUG
			memset(_last, 0, sizeof(_T) * (_back - _last));
#endif
		}
		*_last = _elem;
		++_last;
		return true;
	}
	*_last = _elem;
	++_last;

	return false;
}


// Appends an array of elements to the end of the container
//		buffer:
//			Pointer to the array of elements to be appended
//		_numElem:
//			Number of elements in 'buffer' to be appended
//		NOTE:	_numElem must not be more than the size of buffer!
//				i.e. 'buffer' to 'buffer + _numElem' must be initiallized memory!
template<typename _T>
bool Ring_Buffer<_T>::push(const _T * const buffer, const size_t _numElem)
{
	if (!(_back - _last > _numElem))
	{
		if (capacity() - size() < _numElem)
		{
			reallocate((WRAP_RESIZE_WEIGHT + 1.0f) * _numElem);
			memmove(_last, buffer, sizeof(_T) * _numElem);
			_last += _numElem;
		}
		else {
			memmove(_front, _first, sizeof(_T) * size());
			memmove(_front + size(), buffer, sizeof(_T) * _numElem);
			_last = _front + size() + _numElem;
			_first = _front;

			#ifdef _DEBUG 	// if in debug, set memory to 0
			memset(_last, 0, sizeof(_T) * (_back - _last));
			#endif
		}
		return true;
	}
	memmove(_last, buffer, sizeof(_T) * _numElem);
	_last += _numElem;

	return false;
}


// Removes the first element in the container
template<typename _T>
inline void Ring_Buffer<_T>::pop()
{
	if (_first == _last)
		return;

	#ifdef _DEBUG 	// if in debug, set memory to 0
	memset(_first, 0, sizeof(_T));
	#endif
	++_first;
}


// Removes a number of elements from the front of the container
//		_numElem:
//			Number of elements to remove from the front of the container
template<typename _T>
inline void Ring_Buffer<_T>::pop(const size_t _numElem)
{
#ifdef _DEBUG 	// if in debug, set memory to 0
	memset(_first, 0, sizeof(_T) * _numElem);
#endif
	_first += _numElem;
}


// Removes all elements from the front of the container up to and including the first instance of the terminator
//		term:
//			The terminator of the pop operation, all elements up to and including the first instance of term are removed
//		NOTE: if there is no element equal to the 'term' in the container, no elements are removed
template<typename _T>
void Ring_Buffer<_T>::pop(const _T& term)
{
	_T* termPtr = std::find(_first, _last, term);
	if (termPtr != _last)
	{
#ifdef _DEBUG	// if in debug, set memory to 0
		memset(_first, 0, sizeof(_T) * (termPtr - _first + 1));
#endif
		_first += termPtr - _first + 1;
	}
}


// Clears all elements in the container
template<typename _T>
inline void Ring_Buffer<_T>::clear()
{
#ifdef _DEBUG	// if in debug, set memory to 0
	memset(_first, 0, sizeof(_T)*size());
#endif

	_first = _front;
	_last = _front;
}


// Copies the first element of the container
//		popOff:
//			Determines whether to leave the copied element or whether to remove it.
//		NOTE: returns the default element '_T()' if container is empty, consider testing if empty before use.
template<typename _T>
inline _T Ring_Buffer<_T>::pull(bool popOff)
{
	if (_first == _last)
		return _T();
	
	_T output = *_first;
	if (popOff)
		pop();

	return output;
}


// Copies the first '_numElem' elements of the container
//		_numElem:
//			The number of elements to copy from the front of the container
//		popOff:
//			Determines whether to leave the copied elements or whether to remove them.
//		NOTE: if the number of elements requested is larger than the number of elements in the container
//					then the function returns nullptr, consider checking the return value.
//		WARNING: Must delete the returned buffer after use using delete[]
template<typename _T>
_T* Ring_Buffer<_T>::pull(const size_t _numElem, bool popOff)
{
	if (_numElem > size())
		return nullptr;
	
	_T* const buffer = new _T[_numElem];
	memmove(buffer, _first, sizeof(_T) * _numElem);

	if (popOff)
	{
		pop(_numElem);
	}

	return buffer;
}


// Copies the elements in the container up to and including the terminator
//		term:
//			The terminator, the last element to be copied
//		popOff:
//			Determines whether to leave the copied elements or whether to remove them.
//		NOTE: if there is no element equal to 'term' in the container, then nullptr is returned;
//					consider checking the return value.
//		WARNING: Must delete the returned buffer after use using delete[]
template<typename _T>
_T* Ring_Buffer<_T>::pull(const _T& term, bool popOff)
{
	_T* termPtr = std::find(_first, _last, term);
	if (termPtr == _last)
		return nullptr;

	_T* const buffer = new _T[termPtr - _first + 1];
	memmove(buffer, _first, sizeof(_T) * (termPtr - _first + 1));

	if (popOff)
	{
		#ifdef _DEBUG	// if in debug, set memory to 0
		memset(_first, 0, sizeof(_T) * (termPtr - _first + 1));
		#endif
		_first += termPtr - _first + 1;
	}

	return buffer;
}


// Returns an iterator to the first instance of 'elem' in the container.
//		elem:
//			Value to search for in the container
//		NOTE: if there is no element in the container equal to 'elem', the function
//					returns an iterator just past the end of the container
template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::find(const _T& elem)
{
	return iterator(std::find(_first, _last, elem));
}


template<typename _T>
inline Ring_Buffer<_T>::iterator::iterator() :
	_ptr(nullptr)
{}


template<typename _T>
inline Ring_Buffer<_T>::iterator::iterator(const _T* ptr) :
	_ptr(ptr)
{}

template<typename _T>
inline Ring_Buffer<_T>::iterator::iterator(const Ring_Buffer<_T>::iterator& itr) :
	_ptr(itr._ptr)
{}


template<typename _T>
inline Ring_Buffer<_T>::iterator::~iterator()
{}

template<typename _T>
inline Ring_Buffer<_T>::iterator::operator const _T*() const
{
	return _ptr;
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator & Ring_Buffer<_T>::iterator::operator=(const iterator& itr)
{
	_ptr = itr._ptr;
	return *this;
}

template<typename _T>
inline const _T& Ring_Buffer<_T>::iterator::operator*() const
{
	return *_ptr;
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::iterator::operator+(const int& rhs) const
{
	return iterator(_ptr + rhs);
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::iterator::operator-(const int& rhs) const
{
	return iterator(_ptr - rhs);
}

template<typename _T>
inline int Ring_Buffer<_T>::iterator::operator-(const iterator& rhs) const
{
	return _ptr - rhs._ptr;
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator& Ring_Buffer<_T>::iterator::operator++()
{
	++_ptr;
	return *this;
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::iterator::operator++(int)
{
	return iterator(_ptr++);
}

template<typename _T>
inline typename Ring_Buffer<_T>::iterator& Ring_Buffer<_T>::iterator::operator--()
{
	--_ptr;
	return *this;
}

template<typename _T>
typename Ring_Buffer<_T>::iterator Ring_Buffer<_T>::iterator::operator--(int)
{
	return iterator(_ptr--);
}

template<typename _T>
inline bool Ring_Buffer<_T>::iterator::operator<(const iterator & rhs) const
{
	return _ptr < rhs._ptr;
}

template<typename _T>
inline bool Ring_Buffer<_T>::iterator::operator>(const iterator & rhs) const
{
	return _ptr > rhs._ptr;
}

template<typename _T>
inline bool Ring_Buffer<_T>::iterator::operator==(const iterator & rhs) const
{
	return _ptr == rhs._ptr;
}

template<typename _T>
inline bool Ring_Buffer<_T>::iterator::operator<=(const iterator & rhs) const
{
	return _ptr <= rhs._ptr;
}

template<typename _T>
inline bool Ring_Buffer<_T>::iterator::operator>=(const iterator & rhs) const
{
	return _ptr >= rhs._ptr;
}

#endif