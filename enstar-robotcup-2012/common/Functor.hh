#ifndef FUNCTOR_HH
#define FUNCTOR_HH

template< class Class >
class Functor
{
	public:
		Functor(Class* anInstance = NULL, void* (Class::*aFunction)(void*) = NULL, void* someArgs = NULL);
		virtual ~Functor();

		void* start();

		int startThreaded( pthread_t* thr, const pthread_attr_t* attr );

		static void* apply( void* aFunctor );

	private:
		Class* instance;
		void* (Class::*function)( void* );
		void* args;
};

template< class Class >
class DestroyFunctor : public Functor< Class >
{
	public:
		DestroyFunctor(
				Class* anInstance,
				void* (Class::*aFunction)(void*),
				void* someArgs,
				bool threadedLaunch = false,
				pthread_t* thr = NULL,
				pthread_attr_t* attr = NULL);

		~DestroyFunctor();

	private:
		bool threaded;
		pthread_t* thread;
		pthread_attr_t* attribute;
};

#include "Functor.tem"

#endif // FUNCTOR_HH
