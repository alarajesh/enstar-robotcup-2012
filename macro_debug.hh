#include <iostream>

// DEBUGGER macro
#ifdef DEBUG
	#define DEBUGGER(string) \
		std::cout << "In file : "<< __FILE__ \
		<< " at line : " << __LINE__ \
		<< " : " << (string) << std::endl
#else
	#define DEBUGGER(string)
#endif

// ASSERT macro
#ifdef DEBUG
	#define ASSERT(condition, iffalse) \
		do { \
			if (!( condition )) \
			{ \
				DEBUGGER("ASSERTION FAILED : "); \
				DEBUGGER(iffalse); \
			} \
		} while(0)
#else
	#define ASSERT(condition, iffalse)
#endif

// WHENDEBUG macro
#ifdef DEBUG
	#define WHENDEBUG(somethingtodo) \
			somethingtodo
#else
	#define WHENDEBUG(somethingtodo)
#endif

// TEST macro
#ifdef ONTEST
	#define TEST(somethingtodo) \
		do { \
			somethingtodo \
		} while(0)
#else
	#define TEST(somethingtodo)
#endif
