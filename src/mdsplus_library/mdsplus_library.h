#ifndef MDSPLUS_LIBRARY_H
#define MDSPLUS_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

// Establish a connection to the MDSplus server
// Returns a pointer to the connection object, or NULL on failure
void* establish_connection(const char* server);

// Open a tree on the server
// Returns 0 on success, -1 on failure
int open_tree(void* connection, const char* tree_name, int shot);

// Evaluate an expression and retrieve data and time arrays
// Returns 0 on success, -1 on failure
// Allocates memory for data and time arrays, which must be freed by free_signal_data
int get_signal_data(void* connection, const char* expression, float** data, int* data_size, float** time, int* time_size);

// Free allocated memory for data and time arrays
void free_signal_data(float* data, float* time);

// Close the connection and clean up
void close_connection(void* connection);

#ifdef __cplusplus
}
#endif

#endif // MDSPLUS_LIBRARY_H