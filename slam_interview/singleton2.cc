#include <iostream>
#include <stdlib.h>
#include <mutex>
using namespace std;

// 泛型编程
template<typename T>
class Singleton{
    // friend class T;
public:
    // C++11 static magin 当静态成员初始化时 并发，并且只有一个进行初始化操作，其余线程阻塞
    static T* GetInstance() { 
        static T _instance;
        return &_instance;
    }
    void print_info() {
        cout << " object create " << endl;
    }
protected:
    ~Singleton() { cout << " ~Singleton " << endl;}
    Singleton() {cout << " Singleton " << endl;}
    Singleton(const Singleton& s1) {} 
    // Singleton& operator= (const Singleton&) {}
};
class desiginSingleton : public Singleton<desiginSingleton> {
    friend class Singleton<desiginSingleton>;
private:
    ~desiginSingleton() {cout << " ~designsingleton " << endl;}
    desiginSingleton() {cout << " designsingleton " << endl;}
    desiginSingleton(const desiginSingleton&) {}
public:
    void print_info() {
        cout << " desiginSingleton create " << endl;
    }

};
int main() {
    desiginSingleton::GetInstance(); // 全局访问点
    return 0;
}