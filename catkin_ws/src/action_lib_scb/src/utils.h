#ifndef __UTILS_H__
#define __UTILS_H__

/**
* http://stackoverflow.com/a/4609795
*/
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

const double INF = 99999999;

#endif
