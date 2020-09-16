#include <string>
#include <vector>
using namespace std;

class Foo
{
public:
    Foo(int n, int m) : n_(n), m_(m), a(1), b(2), c(3), d(4){};
    virtual ~Foo() = default;
    int a;
    int b;
    void Bar()
    {
        bar();
    }
    virtual void bar()
    {
        printf("Foo -------> bar\n");
    }

protected:
    int c;
    int d;

private:
    int n_;
    int m_;
};

class MyFoo : public Foo
{
public:
    MyFoo(int n, int m) : Foo(n, m){};
    void bar()
    {
        printf("MyFoo ---------> bar\n");
    }

protected:
};

int main()
{
    Foo foo(5, 6);
    MyFoo mfoo(5, 6);
    foo.Bar();
    mfoo.Bar();
    mfoo.bar();
    Foo *pfoo = &mfoo;
    pfoo->Bar();
    mfoo.Foo::Bar();
    mfoo.Foo::bar();
}