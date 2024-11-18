#ifndef LIFE_LIB_HPP
#define LIFE_LIB_HPP

#ifdef _WIN32
#ifdef LIFELIB_EXP
#define LIFELIB_API __declspec(dllexport)
#else
#define LIFELIB_API __declspec(dllimport)
#endif
#else
#define LIFELIB_API
#endif //!_WIN32

#endif // !LIFE_LIB_HPP