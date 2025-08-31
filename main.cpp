#include <fstream>
#include <sstream>
#include <sched.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <ctime>
#include <set>
#include <chrono>
#include <typeinfo>
#include <pthread.h>
#include <atomic>
#include <cassert>
#include <csignal>
#include <vector>
#include <algorithm>
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include "avl.h"
#include "node.h"
#include "record_manager.h"

using namespace std;

typedef long long test_type;
typedef long long V;

typedef record_manager<reclaimer_debra<test_type>, allocator_new<test_type>, pool_none<test_type>, Node<test_type,V>> RECORD_MANAGER_T;
typedef AVL<test_type, V, std::less<test_type>, RECORD_MANAGER_T> DATA_STRUCTURE_T;

DATA_STRUCTURE_T *ds;

#define printpair(a) { auto _ = (a); cout << "result : (" << _.first << ", " << _.second << ")" << endl; }

void simpleTest() {
    ds->insert(0, 10, 99);
    ds->insert(0, 20, 88);
    ds->insert(0, 1, 111);
    ds->erase(0, 10);
    ds->erase(0, 10);
    ds->erase(0, 1);
    ds->erase(0, 1);
    ds->erase(0, 20);
    ds->erase(0, 20);

    auto x = ds->find(0, 10);
    printpair(x);
    x = ds->find(0, 20);
    printpair(x);
    x = ds->find(0, 1);
    printpair(x);
    x = ds->find(0, 999);
    printpair(x);
    x = ds->find(0, -1);
    printpair(x);
}

void stressTest() {
    const int N = 50000;
    vector<int> num;
    long long sum = 0;
    for(int i = 1; i < N; ++i) {
        num.push_back(i);
        num.push_back(i);
    }
    random_shuffle(num.begin(), num.end());
    vector<bool> found(N);
    for(int x : num) {
        if(!found[x]) {
            found[x] = true;
            ds->insert(0, x, x+1);
            sum += x;
        } else {
            auto result = ds->erase(0, x);
            assert(result.second);
            long long asum = ds->debugKeySum();
            //cout << "sum = " << sum-x << ", actual sum = " << asum << ", size = " << ds->size() << endl;
            assert(sum-x == asum);
            sum -= x;
        }
    }
}

void linearTest() {
    const int N = 100000;
    // random deletion
    vector<int> num;
    long long sum = 0;
    for(int x = 1; x < N; ++x) {
        num.push_back(x);
        sum += x;
    }
    random_shuffle(num.begin(), num.end());
    vector<int> num2;
    for(int x = 1; x < N; ++x) {
        num2.push_back(x);
    }
    random_shuffle(num2.begin(), num2.end());

    // linear test (uncomment the next line for random test)
    for(int x = 1; x < N; x += 1) {
    //for(int x : num2) {
        ds->insert(0, x, x+1);
    }
    
    int count = 0;
    for(int x : num) {
        //cout << "count = " << ++count << ", x = " << x << endl;
        auto result = ds->erase(0, x);
        assert(result.second);
        //cout << "sum = " << sum-x << ", actual sum = " << ds->debugKeySum() << endl;
        assert(sum-x == ds->debugKeySum());
        sum -= x;
    }
    //printpair(ds->erase(0, 100));
    //printpair(ds->erase(0, 10));
    //printpair(ds->erase(0, 50));
}

int main() {
    const int allowedViolationsPerPath = 10;
    const int NO_KEY = -1;
    const int NO_VALUE = -1;
    const int numIterationsPerFixing = 10;

    ds = new DATA_STRUCTURE_T(NO_KEY, NO_VALUE, 1, SIGQUIT, allowedViolationsPerPath, numIterationsPerFixing);
    ds->initThread(0);
    
    //simpleTest();
    //linearTest();
    stressTest();
    
    //ds->getRoot()->printTreeFile(cout);
    cout << endl;
    cout << "success" << endl;
    cout << "height = " << ds->height() << endl;
    cout << "size = " << ds->size() << endl;
    cout << "debugKeySum = " << ds->debugKeySum() << endl;
    return 0;
}