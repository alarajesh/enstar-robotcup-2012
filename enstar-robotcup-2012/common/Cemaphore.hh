#ifndef CEMAPHORE_HH
#define CEMAPHORE_HH

template < typename type >
class Cemaphore
{
	public:
		Cemaphore();

		type operator=( const type& tp_ );

	private:
		type m_tp;
};

#include "cemaphore.tem"

#endif // CEMAPHORE_HH
