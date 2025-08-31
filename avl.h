#ifndef AVL_H
#define AVL_H

#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <unistd.h>
#include <sys/types.h>
#include <stdexcept>
#include <bitset>

#include "node.h"
#include "plaf.h"
#include "gstats.h"
#include "scxrecord.h"
#include "debugprinting.h"
#include "globals.h"

using namespace std;

template <class K, class V, class Compare, class MasterRecordMgr>
class AVL {
private:
    Compare cmp;
    
    /**
     * @brief Memory Management
     * 
     */
    MasterRecordMgr * const recordmgr;
    
    /**
     * @brief TODO: SCX Stuffs
     * (reclaim scx, etc etc)
     */

    /**
     * AVL Tree Implementation
     */
    const int N; // number of violations to allow on a search path before we fix everything on it
    const int numIterationsPerFixing;
    Node<K,V> *root; // actually const

    bool updateInsert(const int tid, const K& key, const V& val, const bool onlyIfAbsent, V *result, bool *shouldRebalance); // the last 2 args are output args
    bool updateErase(const int tid, const K& key, V *result, bool *shouldRebalance); // the last 2 args are output args
    bool updateRebalancingStep(const int tid, const K& key);
    
    // rotations
    //TODO
    void fixAllToKey(const int tid, const K& k);
    bool doN(const int, Node<K,V> **, void **, bool);
    bool doNN1(const int, Node<K,V> **, void **, bool);
    bool doNN2(const int, Node<K,V> **, void **, bool);
    bool doNN3(const int, Node<K,V> **, void **, bool);
    bool doNN4(const int, Node<K,V> **, void **, bool);
    bool doN_(const int, Node<K,V> **, void **, bool);
    bool doN_P1(const int, Node<K,V> **, void **, bool);
    bool doN_P2(const int, Node<K,V> **, void **, bool);
    bool doN_P3(const int, Node<K,V> **, void **, bool);
    bool doN_P4(const int, Node<K,V> **, void **, bool);
    bool doP(const int, Node<K,V> **, void **, bool);
    bool doPP0(const int, Node<K,V> **, void **, bool);
    bool doPP1(const int, Node<K,V> **, void **, bool);
    bool doPP2(const int, Node<K,V> **, void **, bool);
    bool doPP3(const int, Node<K,V> **, void **, bool);
    bool doPP4(const int, Node<K,V> **, void **, bool);
    bool doP_(const int, Node<K,V> **, void **, bool);
    bool doP_N0(const int, Node<K,V> **, void **, bool);
    bool doP_N1(const int, Node<K,V> **, void **, bool);
    bool doP_N2(const int, Node<K,V> **, void **, bool);
    bool doP_N3(const int, Node<K,V> **, void **, bool);
    bool doP_N4(const int, Node<K,V> **, void **, bool);

    int init[MAX_THREADS_POW2] = {0,}; 

public:
    const K NO_KEY;
    const V NO_VALUE;
    AVL(const K& _NO_KEY,
        const V& _NO_VALUE,
        const int numProcesses,
        int neutralizeSignal,
        int allowedViolationsPerPath = 6,
        int _numIterationsPerFixing = 10);

    /**
     * This function must be called once by each thread that will
     * invoke any functions on this class.
     * 
     * It must be okay that we do this with the main thread and later with another thread!!!
     */
    void initThread(const int tid);
    void deinitThread(const int tid);
    
    void dfsDeallocateBottomUp(Node<K,V> *const u, set<void*> &seen, int *numNodes) {
        if (u == NULL) return;
        if ((Node<K,V>*) u->left.load(memory_order_relaxed) != NULL) {
            dfsDeallocateBottomUp((Node<K,V>*) u->left.load(memory_order_relaxed), seen, numNodes);
            dfsDeallocateBottomUp((Node<K,V>*) u->right.load(memory_order_relaxed), seen, numNodes);
        }
        
        ++(*numNodes);
        recordmgr->deallocate(0 /* tid */, u);
    }
    
    ~AVL() {
        // free every node and scx record currently in the data structure.
        // an easy DFS, freeing from the leaves up, handles all nodes.
        // cleaning up scx records is a little bit harder if they are in progress or aborted.
        // they have to be collected and freed only once, since they can be pointed to by many nodes.
        // so, we keep them in a set, then free each set element at the end.
        
        set<void*> seen;
        int numNodes = 0;
        dfsDeallocateBottomUp(root, seen, &numNodes);
        delete recordmgr;
    }
    
    Node<K,V> *getRoot(void) { return root; }
    const V insert(const int tid, const K& key, const V& val);
    const V insertIfAbsent(const int tid, const K& key, const V& val);
    const pair<V,bool> erase(const int tid, const K& key);
    const pair<V,bool> find(const int tid, const K& key);
    bool contains(const int tid, const K& key);
    int size(void); // Linear time operation
    int height(void); // Linear time operation

    /**
     * Debugging functions
     */
    long long debugKeySum(Node<K,V> * node);
    long long debugKeySum() {
        return debugKeySum((Node<K,V> *) root->left.load(memory_order_relaxed));
    }

    int computeSize(Node<K,V>* node);
    
    int computeHeight(Node<K,V>* node);

    void debugPrintAllocatorStatus() {
        recordmgr->printStatus();
    }
    void debugPrintToFile(string prefix, long id1, string infix, long id2, string suffix) {
        stringstream ss;
        ss<<prefix<<id1<<infix<<id2<<suffix;
        COUTATOMIC("print to filename \""<<ss.str()<<"\""<<endl);
        fstream fs (ss.str().c_str(), fstream::out);
        root->printTreeFile(fs);
        fs.close();
    }
    void debugPrintToFileWeight(string prefix, long id1, string infix, long id2, string suffix) {
        stringstream ss;
        ss<<prefix<<id1<<infix<<id2<<suffix;
        COUTATOMIC("print to filename \""<<ss.str()<<"\""<<endl);
        fstream fs (ss.str().c_str(), fstream::out);
        root->printTreeFileWeight(fs);
        fs.close();
    }
    /*
    void clearCounters() {
        //counters->clear();
        //recordmgr->clearCounters();
    }
    debugCounters * const debugGetCounters() {
        return counters;
    }
    */
    MasterRecordMgr * const debugGetRecordMgr() {
        return recordmgr;
    }

};

#include "avl_impl.h"

#endif