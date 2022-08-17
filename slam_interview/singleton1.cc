#include <iostream>
#include <stdlib.h>

using namespace std;

class Singleton{
public:
    static Singleton* GetInstance() { // 存放在堆区 不能调到析构函数（析构私有化，外面无法调用）
        if (_instance == nullptr) {
            _instance = new Singleton();
            atexit(deconstructor); // 主动调用delete, 对象被析构，防止内存泄漏
        }
        return _instance;
    }
    void print_info() {
        cout << " object create " << endl;
    }
private:
    static void deconstructor() {
        if (_instance != nullptr) {
            cout << " test " << endl;
            delete[] _instance; // array new 搭配 array delete
            _instance = nullptr;
        }
    }
    ~Singleton() { cout << " ~Singleton " << endl;}
    Singleton() {}
    Singleton(const Singleton& s1) {}
    // Singleton& operator= (const Singleton&) {}
    static Singleton* _instance; // 位于静态全局区 对象自己释放
};
Singleton* Singleton::_instance = nullptr;

int main() {
    Singleton::GetInstance()->print_info(); // 全局访问点
    return 0;
}