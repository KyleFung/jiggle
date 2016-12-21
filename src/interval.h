#ifndef INTERVAL_H
#define INTERVAL_H

#include <stdexcept>
#include <string>
#include <vector>

template <class T> class interval {
  public:
    interval();
    interval(std::vector<T>* base, std::vector<int>* index, int start, int end);

    T& getItem(int i);
    T& operator[](int i);

    // Should only ever get called from root interval
    T& interval<T>::getItemFromBase(int i);
    void addItem(T item);
    void cleanUp();

    int size();
    bool empty();

  private:
    std::vector<T>* mBaseList;
    std::vector<int>* mIndexList;
    int mStart;
    int mEnd;

    void checkRoot();
};

template<typename T>
interval<T>::interval() {
    mBaseList = new std::vector<T>();
    mIndexList = new std::vector<int>();
    mStart = 0;
    mEnd = -1;
}

template <typename T>
interval<T>::interval(std::vector<T>* base, std::vector<int>* index, int start, int end) :
mBaseList(base), mIndexList(index), mStart(start), mEnd(end) {}

template <typename T>
void interval<T>::checkRoot() {
    if (size() != mIndexList->size())
        throw std::string("Only add to master list!");
}

template <typename T>
void interval<T>::addItem(T item) {
    checkRoot();
    mBaseList->push_back(item);
    mIndexList->push_back(mIndexList->size());
    mEnd++;
}

template <typename T>
T& interval<T>::getItemFromBase(int i) {
    checkRoot();
    return (*mBaseList)[i];
}

template <typename T>
void interval<T>::cleanUp() {
    checkRoot();
    delete mBaseList;
    delete mIndexList;
}

template <typename T>
T& interval<T>::operator[](int i) {
    return getItem(i);
}

template <typename T>
T& interval<T>::getItem(int i) {
    if (i < 0 || i >= size()) {
        throw std::out_of_range("Element is out of range");
    }
    int index = (*mIndexList)[i + mStart];
    return getItemFromBase(index);
}

template <typename T>
inline int interval<T>::size() {
    return mEnd - mStart + 1;
}

template <typename T>
bool interval<T>::empty() {
    return (size() <= 0);
}

#endif
