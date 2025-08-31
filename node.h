#ifndef NODE_H
#define NODE_H

#include<iostream>
#include<iomanip>
#include<atomic>
#include<set>
// #include "scxrecord.h" // TODO: Read SCX/LLX repo/code

using namespace std;

template<class K, class V>
class Node {
public:
    int tag; // tag
    int b; // relaxed balance factor
    V value;
    K key;
    atomic_uintptr_t left; // an unsigned integer type that is capable of storing a data pointer, 
    // A common reason to want an integer type that can hold an architecture's pointer type is 
    // to perform integer-specific operations on a pointer, 
    // or to obscure the type of a pointer by providing it as an integer "handle".
    atomic_uintptr_t right;
    /*
    atomic_uintptr_t scxRecord;
    atomic_bool marked; // might be able to combine this elegantly with scx record pointer... (maybe we can piggyback on the version number mechanism, using the same bit to indicate ver# OR marked)
    */
    
    Node() {
        // left blank for efficiency with custom allocator
    }
    Node(const Node& node) {
        // left blank for efficiency with custom allocator
    }
    
    K getKey() {
        return key;
    }
    V getValue() {
        return value;
    }
    
    friend ostream& operator<<(ostream& os, const Node<K,V>& obj) {
        ios::fmtflags f( os.flags() );
        os<<"[key="<<obj.key
          <<" tag="<<obj.tag
          <<" b="<<obj.b;
          //<<" marked="<<obj.marked.load(memory_order_relaxed);
        //os<<" scxRecord@0x"<<hex<<(long)(obj.scxRecord.load(memory_order_relaxed));
//        os.flags(f);
        os<<" left@0x"<<hex<<(long)(obj.left.load(memory_order_relaxed));
//        os.flags(f);
        os<<" right@0x"<<hex<<(long)(obj.right.load(memory_order_relaxed));
//        os.flags(f);
        os<<"]"<<"@0x"<<hex<<(long)(&obj);
        os.flags(f);
        return os;
    }

    // somewhat slow version that detects cycles in the tree
    void printTreeFile(ostream& os, set< Node<K,V>* > *seen) {
//        os<<"(["<<key<<","<</*(long)(*this)<<","<<*/marked<<","<<scxRecord->state<<"],"<<weight<<",";
        //os<<"(["<<key<<","<<marked.load(memory_order_relaxed)<<"],"<<((SCXRecord<K,V>*) scxRecord.load(memory_order_relaxed))->state.load(memory_order_relaxed)<<",";
        os<<"(["<<key<<","<<value<<","<<tag<<","<<b<<"],";
        Node<K,V>* __left = (Node<K,V>*) left.load(memory_order_relaxed);
        Node<K,V>* __right = (Node<K,V>*) right.load(memory_order_relaxed);
        if (__left == NULL) {
            os<<"-";
        } else if (seen->find(__left) != seen->end()) {   // for finding cycles
            os<<"!"; // cycle!                          // for finding cycles
        } else {
            seen->insert(__left);
            __left->printTreeFile(os, seen);
        }
        os<<",";
        if (__right == NULL) {
            os<<"-";
        } else if (seen->find(__right) != seen->end()) {  // for finding cycles
            os<<"!"; // cycle!                          // for finding cycles
        } else {
            seen->insert(__right);
            __right->printTreeFile(os, seen);
        }
        os<<")";
    }

    void printTreeFile(ostream& os) {
        set< Node<K,V>* > seen;
        printTreeFile(os, &seen);
    }
    
    // somewhat slow version that detects cycles in the tree
    void printTreeFileWeight(ostream& os, set< Node<K,V>* > *seen) {
//        os<<"(["<<key<<","<</*(long)(*this)<<","<<*/marked<<","<<scxRecord->state<<"],"<<weight<<",";
        os<<"(["<<key<<","<<tag<<","<<b<<"],";
        Node<K,V>* __left = (Node<K,V>*) left.load(memory_order_relaxed);
        Node<K,V>* __right = (Node<K,V>*) right.load(memory_order_relaxed);
        if (__left == NULL) {
            os<<"-";
        } else if (seen->find(__left) != seen->end()) {   // for finding cycles
            os<<"!"; // cycle!                          // for finding cycles
        } else {
            seen->insert(__left);
            __left->printTreeFileWeight(os, seen);
        }
        os<<",";
        if (__right == NULL) {
            os<<"-";
        } else if (seen->find(__right) != seen->end()) {  // for finding cycles
            os<<"!"; // cycle!                          // for finding cycles
        } else {
            seen->insert(__right);
            __right->printTreeFileWeight(os, seen);
        }
        os<<")";
    }

    void printTreeFileWeight(ostream& os) {
        set< Node<K,V>* > seen;
        printTreeFileWeight(os, &seen);
    }
};

#endif