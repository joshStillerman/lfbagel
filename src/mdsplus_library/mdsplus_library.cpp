#include "mdsplus_library.h"
#include <mdsobjects.h>
#include <cstring>
#include <stdexcept>

using namespace MDSplus;

extern "C" {


void* establish_connection(const char* server) {
    try {
        Connection* conn = new Connection((char *)server);
        return static_cast<void*>(conn);
    } catch (const std::exception& e) {
        return nullptr;
    }
}

int open_tree(void* connection, const char* tree_name, int shot) {
    try {
        Connection* conn = static_cast<Connection*>(connection);
        conn->openTree((char *)tree_name, shot);
        return 0; // Success
    } catch (const std::exception& e) {
        return -1; // Failure
    }
}

void *get_signal(void *connection, const char *expression) {
    try {
        Connection* conn = static_cast<Connection*>(connection);
        Data* result = conn->get(expression);
        if (!result) {
            return nullptr; // Failure
        }
        return static_cast<void*>(result);
    } catch (const std::exception& e) {
        return nullptr; // Failure
    }
}

void *get_signal_data(void *signal, float **data, int *length) {
    try {
        Connection* conn = static_cast<Connection*>(connection);
        Data* result = conn->get(expression);
        if (!result) {
            return nullptr; // Failure
        }
        float* dataArray = result->getFloatArray();
        *length = result->getLength();
        *data = new float[*length];
        std::memcpy(*data, dataArray, *length * sizeof(float));
        return static_cast<void*>(result);
    } catch (const std::exception& e) {
        return nullptr; // Failure
    }
}
void *get_signal_data(void *connection, const char *expression) {
    try {
        Connection* conn = static_cast<Connection*>(connection);
        Data* result = conn->get(expression);
        if (!result) {
            return nullptr; // Failure
        }
        return static_cast<void*>(result);
    } catch (const std::exception& e) {
        return nullptr; // Failure
    }
}
int get_signal_data(void* connection, const char* expression, float** data, int* data_size, float** time, int* time_size) {
    try {
        Connection* conn = static_cast<Connection*>(connection);
        Data* result = conn->get(expression);
        if (!result) {
            return -1; // Failure
        }
        float* dataArray = result->getFloatArray();
        float* timeArray = result->Dimensions[0]->getFloatArray();
        data_size = result->getLength();
        time_size = result->Dimensions[0]->getLength();
        return 0; // Success
    } catch (const std::exception& e) {
        return -1; // Failure
    }
}

void free_signal_data(float* data, float* time) {
    delete[] data;
    delete[] time;
}

void close_connection(void* connection) {
    Connection* conn = static_cast<Connection*>(connection);
    delete conn;
}

}