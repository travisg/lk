extern "C"
{

// https://cplusplus.github.io/CWG/issues/1766.html
// It is only since C++17 that loading an out-of-range value for an enum is undefined behavior.
// But clang treats this as UB even with C++14(LK compiles C++ files with --std=c++14 by default)
void load_invalid_value_enum(void) {
    enum E {e1, e2, e3, e4};
    volatile int a = 5;
    volatile enum E e = *((volatile enum E*)(&a));
    (void) &e;
}
}
