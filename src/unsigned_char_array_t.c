#include <stdio.h>
#include <stdlib.h>
#include "unsigned_char_array_t.h"


unsigned_char_array_t* unsigned_char_array_constructor(size_t length) {
    unsigned_char_array_t* result = (unsigned_char_array_t*) malloc(sizeof(unsigned_char_array_t));
    result->data = (unsigned char *) calloc(length, sizeof(unsigned char));
    result->length = length;
    return result;
}

void unsigned_char_array_destructor(void* array) {
    free(((unsigned_char_array_t*) array)->data);
    free(array);
}

void* unsigned_char_array_copy_constructor(void* array) {
    unsigned_char_array_t* source = (unsigned_char_array_t*) array;
    unsigned_char_array_t* copy = (unsigned_char_array_t*) malloc(sizeof(unsigned_char_array_t));
    copy->data = (unsigned char *) calloc(source->length, sizeof(unsigned char));
    copy->length = source->length;
    for (size_t i = 0; i < source->length; i++) {
        copy->data[i] = source->data[i];
    }
    return (void*) copy;
}
