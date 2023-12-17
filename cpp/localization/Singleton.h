#ifndef SINGLETON_H
#define SINGLETON_H  

template <class T>
class Singleton {
public:

    static T& getInstance() {
        static T instance;
        return instance;
    }

private:
    Singleton();
    ~Singleton();
    Singleton(Singleton const&);
    Singleton& operator=(Singleton const&);
};

#endif // SINGLETON_H
