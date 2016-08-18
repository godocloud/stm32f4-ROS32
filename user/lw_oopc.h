/*  lw_oopc.h */
#ifndef LOOPC_H
#define LOOPC_H
#include <stdlib.h>
#define CLASS(type)\
typedef struct type type; \
struct type
#define CTOR(type) \
void* type##Setting(type*); \
void* type##New() \
       { \
         struct type *t; \
         t = (struct type *)malloc(sizeof(struct type)); \
         return type##Setting(t); \
       } \
void* type##Setting(type *t) \
      {
#define CTOR2(type, type2) \
void* type2##Setting(type2*); \
void* type2##New() \
       { \
        struct type *t; \
	t = (struct type *)malloc(sizeof(struct type)); \
        return type2##Setting(t); \
       } \
void* type2##Setting(type *t) \
        {
#define END_CTOR return (void*)t;  }
#define FUNCTION_SETTING(f1, f2)  t->f1 = f2;
#define IMPLEMENTS(type) type type
#define INTERFACE(type) \
typedef struct type type; \
struct type
#endif
/*     end     */
