#ifndef UNSIGNED_CHAR_ARRAY_T_H_I
#define UNSIGNED_CHAR_ARRAY_T_H_I

  typedef struct unsigned_char_array_t {
    unsigned char * data;
    size_t length;
  } unsigned_char_array_t;

  unsigned_char_array_t* unsigned_char_array_constructor(size_t length);
  void unsigned_char_array_destructor(void* array);
  void* unsigned_char_array_copy_constructor(void* array);

#endif
