/* Tests stay active under NDEBUG and exit without a platform crash dialog. */
#pragma once
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(expression) do { if (!(expression)) { \
    fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #expression); \
    fflush(stderr); exit(86); } } while (0)
