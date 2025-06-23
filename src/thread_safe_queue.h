#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

        #include <queue>
        #include <mutex>
        #include <atomic>

        template <class T>
        class thread_safe_queue
        {
        public:

            thread_safe_queue()
                : _queue()
                , _mutex()
                , _size(0)
            { }

            ~thread_safe_queue() = default;

            void push(T value)
            {
                _mutex.lock();
                _queue.push(value);
                ++_size;
                _mutex.unlock();
            }

            T pop()
            {
                T value = {};

                _mutex.lock();
                value = _queue.front();
                _queue.pop();
                --_size;
                _mutex.unlock();

                return value;
            }

            size_t size() const
            {
                return _size;
            }

            bool empty() const
            {
                return (_size == 0);
            }

        private:

            std::queue<T> _queue;
            std::mutex _mutex;
            std::atomic_size_t _size;

        }; // thread_safe_queue<T>
#endif
