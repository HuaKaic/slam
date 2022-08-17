# 设计模式
## 知识点引入：
1. 类模型：继承、组合、*聚合*、虚函数重载
***
```
// 继承
class son : public father {
}; 内存顺序：father\son
// 组合
class son1 {
private: father fa;
}; 内存顺序：father\son1
// 聚合（设计模式选择模式）
class son2 {
    friend son1 // son1 可以访问son2的所有属性值，友元具有单向性
private: father* fa;
}; 内存顺序：son2 -> father->[fa1, fa2, fa3]
// 虚函数重载
class father {
    virtual void cook(){};
    virtual void drive(){};
    int a;
} 内存顺序：虚函数指针-> 虚函数表[cook, drive] -> a;
// 继承时覆盖虚函数
class son : public father{
    virtual void drive(){}；
    virtual void code(){};
    int b;
} 内存顺序；虚函数指针->[cook, son::drive, son:: code]
```
2.设计原则
拓展开放修改封闭原则、单一职责原则（一个类只有一个引起改变的变量）、里氏覆盖原则（虚函数覆盖时应该保留父类的功能）、接口隔离原则（属性分类，public\private\protected）、组合优于继承
***
***
1.单例模式（运行阶段一个类只有一个对象，只有一个**全局访问点**，由于类对象的创建和销毁需要大量资源）
```
// 接口隔离原则
class ZooShow {
public:
    // 固定流程封装
    void show() {
        show1();
        show2();
        show3();
        show4();
    }
protected:
    // 接口隔离，不允许客户调用，允许子类拓展
    virtual show1(){};
    virtual show2(){};
    virtual show3(){}; // 利用继承关系，拓展show1...
}
class ZooShow1 : public ZooShow{
protected:
    virtual show1(){};
}
```
