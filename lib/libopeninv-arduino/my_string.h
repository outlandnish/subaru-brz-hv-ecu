/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef MY_STRING_H
#define MY_STRING_H

#include <string.h>

#ifndef NULL
#define NULL 0L
#endif

#define TOSTR_(...) #__VA_ARGS__
#define STRINGIFY(x) TOSTR_(x)

#ifdef __cplusplus
extern "C"
{
#endif

// Use standard library implementations where possible
static inline int my_strcmp(const char *str1, const char *str2) {
    return strcmp(str1, str2);
}

static inline void my_strcpy(char *str1, const char *str2) {
    strcpy(str1, str2);
}

static inline int my_strlen(const char *str) {
    return strlen(str);
}

static inline void memcpy32(int* target, int *source, int length) {
    memcpy(target, source, length * sizeof(int));
}

static inline void memset32(int* target, int value, int length) {
    for (int i = 0; i < length; i++) {
        target[i] = value;
    }
}

#ifdef __cplusplus
}
#endif
#endif // MY_STRING_H
