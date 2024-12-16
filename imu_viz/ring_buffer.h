#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <QVector>

template <typename T>
class RingBuffer {
public:
    // Default constructor
    RingBuffer()
        : capacity_(0), buffer_(), head_(0), size_(0) {}

    explicit RingBuffer(int capacity)
        : capacity_(capacity), buffer_(capacity), head_(0), size_(0) {}

    void push(const T& value) {
        buffer_[head_] = value;
        head_ = (head_ + 1) % capacity_;
        if (size_ < capacity_) {
            ++size_;
        }
    }

    QVector<T> getLastN(int n) const {
        QVector<T> result;
        int count = std::min(n, size_);
        result.reserve(count);
        for (int i = 0; i < count; ++i) {
            int index = (head_ + capacity_ - count + i) % capacity_;
            result.append(buffer_[index]);
        }
        return result;
    }

    T getMinofLastN(int n) const {
        T ret = std::numeric_limits<double>::max();
        int count = std::min(n, size_);
        for (int i = 0; i < count; ++i) {
            int index = (head_ + capacity_ - count + i) % capacity_;
            ret = std::min(ret, buffer_[index]);
        }
        return ret;
    }

    T getMaxofLastN(int n) const {
        T ret = std::numeric_limits<double>::min();
        int count = std::min(n, size_);
        for (int i = 0; i < count; ++i) {
            int index = (head_ + capacity_ - count + i) % capacity_;
            ret = std::max(ret, buffer_[index]);
        }
        return ret;
    }
    int size() const { return size_; }

    void clear() {
        buffer_.fill(T());
        head_ = 0;
        size_ = 0;
    }

private:
    int capacity_;
    QVector<T> buffer_;
    int head_;
    int size_;

};

#endif // RING_BUFFER_H
