#ifndef __UTILS_H__
#define __UTILS_H__

#define RED '\033[0;31m'
#define NC '\033[0m'

/**
* http://stackoverflow.com/a/4609795
*/
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

const double INF = 99999999;

#endif
