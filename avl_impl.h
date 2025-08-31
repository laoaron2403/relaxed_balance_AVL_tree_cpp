#ifndef AVL_IMPL_H
#define AVL_IMPL_H

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <sys/types.h>
#include "avl.h"
#include "node.h"

//#define memory_order_relaxed memory_order_seq_cst

using namespace std;

#ifdef NOREBALANCING
#define IFREBALANCING if (0)
#else
#define IFREBALANCING if (1)
#endif

/*
#define IF_FAIL_TO_PROTECT_SCX(info, tid, _obj, arg2, arg3) \
    info.obj = _obj; \
    info.ptrToObj = arg2; \
    info.nodeContainingPtrToObjIsMarked = arg3; \
    if (_obj != dummy && !recordmgr->protect(tid, _obj, callbackCheckNotRetired, (void*) &info))
*/
#define IF_FAIL_TO_PROTECT_NODE(info, tid, _obj, arg2, arg3) \
    info.obj = _obj; \
    info.ptrToObj = arg2; \
    info.nodeContainingPtrToObjIsMarked = arg3; \
    if (_obj != root && !recordmgr->protect(tid, _obj, callbackCheckNotRetired, (void*) &info))

/* // for hazard pointer scheme
inline CallbackReturn callbackCheckNotRetired(CallbackArg arg) {
    Chromatic_retired_info *info = (Chromatic_retired_info*) arg;
    if ((void*) info->ptrToObj->load(memory_order_relaxed) == info->obj) {
        // we insert a compiler barrier (not a memory barrier!)
        // to prevent these if statements from being merged or reordered.
        // we care because we need to see that ptrToObj == obj
        // and THEN see that ptrToObject is a field of an object
        // that is not marked. seeing both of these things,
        // in this order, implies that obj is in the data structure.
        SOFTWARE_BARRIER;
        if (!info->nodeContainingPtrToObjIsMarked->load(memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}
*/

#define allocateSCXRecord(tid) recordmgr->template allocate<SCXRecord<K,V> >((tid))
#define allocateNode(tid) recordmgr->template allocate<Node<K,V> >((tid))

#define initializeSCXRecord(_tid, _newop, _type, _nodes, _llxResults, _field, _newNode) { \
    (_newop)->type = (_type); \
    (_newop)->newNode = (_newNode); \
    for (int i=0;i<NUM_OF_NODES[(_type)];++i) { \
        (_newop)->nodes[i] = (_nodes)[i]; \
    } \
    for (int i=0;i<NUM_TO_FREEZE[(_type)];++i) { \
        (_newop)->scxRecordsSeen[i] = (SCXRecord<K,V>*) (_llxResults)[i]; \
    } \
    /* note: synchronization is not necessary for the following accesses, \
       since a memory barrier will occur before this object becomes reachable \
       from an entry point to the data structure. */ \
    (_newop)->state.store(SCXRecord<K,V>::STATE_INPROGRESS, memory_order_relaxed); \
    (_newop)->allFrozen.store(false, memory_order_relaxed); \
    (_newop)->field = (_field); \
}

#define initializeNode(_tid, _newnode, _key, _value, _tag, _b, _left, _right) \
(_newnode); \
{ \
    (_newnode)->key = (_key); \
    (_newnode)->value = (_value); \
    (_newnode)->tag = (_tag); \
    (_newnode)->b = (_b); \
    /* note: synchronization is not necessary for the following accesses, \
       since a memory barrier will occur before this object becomes reachable \
       from an entry point to the data structure. */ \
    (_newnode)->left.store((uintptr_t) (_left), memory_order_relaxed); \
    (_newnode)->right.store((uintptr_t) (_right), memory_order_relaxed); \
    /* (_newnode)->scxRecord.store((uintptr_t) dummy, memory_order_relaxed); \
    (_newnode)->marked.store(false, memory_order_relaxed); */ \
}

template<class K, class V, class Compare, class MasterRecordMgr>
AVL<K,V,Compare,MasterRecordMgr>::AVL(const K& _NO_KEY,
            const V& _NO_VALUE,
            //const V& _RETRY,
            const int numProcesses,
            int neutralizeSignal,
            int allowedViolationsPerPath,
            int _numIterationsPerFixing)
    : N(allowedViolationsPerPath),
            NO_KEY(_NO_KEY),
            NO_VALUE(_NO_VALUE),
            //RETRY(_RETRY),
            numIterationsPerFixing(_numIterationsPerFixing),
            recordmgr(new MasterRecordMgr(numProcesses, neutralizeSignal)) {

    VERBOSE DEBUG COUTATOMIC("constructor chromatic"<<endl);
    const int tid = 0;

    /*
    allocatedSCXRecord = new SCXRecord<K,V>*[numProcesses*PREFETCH_SIZE_WORDS];
    */
    //allocatedNodes = new Node<K,V>*[numProcesses*(PREFETCH_SIZE_WORDS+MAX_NODES-1)];
    /*
    for (int tid=0;tid<numProcesses;++tid) {
        GET_ALLOCATED_SCXRECORD_PTR(tid) = NULL;
    }
    */
    cmp = Compare();

    initThread(tid);
    
    auto guard = recordmgr->getGuard(tid);

    /*
    dummy = allocateSCXRecord(tid);
    dummy->type = SCXRecord<K,V>::TYPE_NOOP;
    dummy->state.store(SCXRecord<K,V>::STATE_ABORTED, memory_order_relaxed); // this is a NO-OP, so it shouldn't start as InProgress; aborted is just more efficient than committed, since we won't try to help marked leaves, which always have the dummy scx record...
    */

    // TODO: define allocate node
    root = allocateNode(tid);
    initializeNode(tid, root, NO_KEY, NO_VALUE, 0, 0, NULL, NULL);
}

template<class K, class V, class Compare, class MasterRecordMgr>
long long AVL<K,V,Compare,MasterRecordMgr>::debugKeySum(Node<K,V> * node) {
    if (node == NULL) return 0;
    if ((void*) node->left.load(memory_order_relaxed) == NULL) {
        long long key = (long long) node->key;
        return (key == NO_KEY) ? 0 : key;
    }
    return debugKeySum((Node<K,V> *) node->left.load(memory_order_relaxed))
         + debugKeySum((Node<K,V> *) node->right.load(memory_order_relaxed));
}


template<class K, class V, class Compare, class MasterRecordMgr>
int AVL<K,V,Compare,MasterRecordMgr>::computeSize(Node<K,V> * const root) {
    if (root == NULL) return 0;
    if ((Node<K,V> *) root->left.load(memory_order_relaxed) != NULL) { // if internal node
        return computeSize((Node<K,V> *) root->left.load(memory_order_relaxed))
                + computeSize((Node<K,V> *) root->right.load(memory_order_relaxed));
    } else { // if leaf
        return 1;
//        printf(" %d", root->key);
    }
}

template<class K, class V, class Compare, class MasterRecordMgr>
int AVL<K,V,Compare,MasterRecordMgr>::size() {
    return computeSize((Node<K,V> *) root->left.load(memory_order_relaxed));
}

template<class K, class V, class Compare, class MasterRecordMgr>
int AVL<K,V,Compare,MasterRecordMgr>::computeHeight(Node<K,V> * const root) {
    if (root == NULL) return 0;
    if ((Node<K,V> *) root->left.load(memory_order_relaxed) != NULL) { // if internal node
        return max(computeHeight((Node<K,V> *) root->left.load(memory_order_relaxed)),
                computeHeight((Node<K,V> *) root->right.load(memory_order_relaxed)))+1;
    } else { // if leaf
        return 1;
//        printf(" %d", root->key);
    }
}

template<class K, class V, class Compare, class MasterRecordMgr>
int AVL<K,V,Compare,MasterRecordMgr>::height() {
    return computeHeight((Node<K,V> *) root->left.load(memory_order_relaxed));
}

/**
 * This function must be called once by each thread that will
 * invoke any functions on this class.
 * 
 * It must be okay that we do this with the main thread and later with another thread!!!
 */
template<class K, class V, class Compare, class MasterRecordMgr>
void AVL<K,V,Compare,MasterRecordMgr>::initThread(const int tid) {
    if (init[tid]) return; else init[tid] = !init[tid];

    recordmgr->initThread(tid);
    /*
    if (GET_ALLOCATED_SCXRECORD_PTR(tid) == NULL) {
        REPLACE_ALLOCATED_SCXRECORD(tid);
        for (int i=0;i<MAX_NODES-1;++i) {
            REPLACE_ALLOCATED_NODE(tid, i);
        }
    }
    */
}

template<class K, class V, class Compare, class MasterRecordMgr>
void AVL<K,V,Compare,MasterRecordMgr>::deinitThread(const int tid) {
    if (!init[tid]) return; else init[tid] = !init[tid];

    recordmgr->deinitThread(tid);
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::contains(const int tid, const K& key) {
    pair<V,bool> result = find(tid, key);
    return result.second;
}

template<class K, class V, class Compare, class MasterRecordMgr>
const pair<V,bool> AVL<K,V,Compare,MasterRecordMgr>::find(const int tid, const K& key) {
    pair<V,bool> result;
    // AVL_retired_info info;
    Node<K,V> *p;
    Node<K,V> *l;
    for (;;) {
        // TRACE COUTATOMICTID("find(tid="<<tid<<" key="<<key<<")"<<endl);
        // CHECKPOINT_AND_RUN_QUERY(tid) {
            auto guard = recordmgr->getGuard(tid, true);
            // root is never retired, so we don't need to call
            // protectPointer before accessing its child pointers
            p = root;
            // IF_FAIL_TO_PROTECT_NODE(info, tid, p, &root->left, &root->marked) {
            //     // ++counters[tid]->findRestart;
            //     continue; /* retry */ 
            // }
            // assert(p != root);
            // assert(recordmgr->isProtected(tid, p));
            l = (Node<K,V>*) p->left.load(memory_order_relaxed);
            if (l == NULL) { // Empty Tree
                result = pair<V,bool>(NO_VALUE, false); // no keys in data structure
                return result; // success
            }
            // IF_FAIL_TO_PROTECT_NODE(info, tid, l, &p->left, &p->marked) { 
            //     // ++counters[tid]->findRestart;
            //     continue; /* retry */ 
            // }

            // assert(recordmgr->isProtected(tid, l));
            while ((Node<K,V>*) l->left.load(memory_order_relaxed) != NULL) {
                // TRACE COUTATOMICTID("traversing tree; l="<<*l<<endl);
                // assert(recordmgr->isProtected(tid, p));
                // recordmgr->unprotect(tid, p);
                p = l; // note: the new p is currently protected
                // assert(recordmgr->isProtected(tid, p));
                // assert(p->key != NO_KEY);
                if (cmp(key, p->key)) {
                    // assert(recordmgr->isProtected(tid, p));
                    l = (Node<K,V>*) p->left.load(memory_order_relaxed);
                    // IF_FAIL_TO_PROTECT_NODE(info, tid, l, &p->left, &p->marked) {
                    //     // ++counters[tid]->findRestart;
                    //     continue; /* retry */ 
                    // }
                } else {
                    // assert(recordmgr->isProtected(tid, p));
                    l = (Node<K,V>*) p->right.load(memory_order_relaxed);
                    // IF_FAIL_TO_PROTECT_NODE(info, tid, l, &p->right, &p->marked) { 
                    //     // ++counters[tid]->findRestart;
                    //     continue; /* retry */ 
                    // }
                }
                // assert(recordmgr->isProtected(tid, l));
            }
            // assert(recordmgr->isProtected(tid, l));
            if (key == l->key) {
                // assert(recordmgr->isProtected(tid, l));
                result = pair<V,bool>(l->value, true);
            } else {
                result = pair<V,bool>(NO_VALUE, false);
            }
            return result; // success
        // }
    }
    return pair<V,bool>(NO_VALUE, false);
}

template<class K, class V, class Compare, class MasterRecordMgr>
const V AVL<K,V,Compare,MasterRecordMgr>::insert(const int tid, const K& key, const V& val) {
    //COUTATOMIC("insert (key,value) = ("<<key<<","<<val<<")"<<endl);
    bool onlyIfAbsent = false;
    V result = NO_VALUE;
    bool shouldRebalance = false;
    bool finished = false;
    while(!finished) {
        finished = updateInsert(tid, key, val, onlyIfAbsent, &result, &shouldRebalance);
    }
    IFREBALANCING if (shouldRebalance) {
        fixAllToKey(tid, key);
    }
    return result;
}

template<class K, class V, class Compare, class MasterRecordMgr>
const V AVL<K,V,Compare,MasterRecordMgr>::insertIfAbsent(const int tid, const K& key, const V& val) {
    bool onlyIfAbsent = true;
    V result = NO_VALUE;
    bool shouldRebalance = false;
    bool finished = false;
    while(!finished) {
        finished = updateInsert(tid, key, val, onlyIfAbsent, &result, &shouldRebalance);
    }
    IFREBALANCING if (shouldRebalance) {
        fixAllToKey(tid, key);
    }
    return result;
}

template<class K, class V, class Compare, class MasterRecordMgr>
const pair<V,bool> AVL<K,V,Compare,MasterRecordMgr>::erase(const int tid, const K& key) {
    V result = NO_VALUE;
    bool shouldRebalance = false;
    bool finished = false;
    while (!finished) {
        auto guard = recordmgr->getGuard(tid);
        finished = updateErase(tid, key, &result, &shouldRebalance);
        // if (!finished) ++counters[tid]->eraseRestart;
    }
    // call another routine to handle rebalancing.
    // this is considered to be a whole new operation (possibly many, in fact).
    IFREBALANCING if (shouldRebalance) {
        fixAllToKey(tid, key);
    }
    return pair<V,bool>(result, (result != NO_VALUE));
}

template<class K, class V, class Compare, class MasterRecordMgr>
void AVL<K,V,Compare,MasterRecordMgr>::fixAllToKey(const int tid, const K& key) {
    int x = numIterationsPerFixing;
    while(x--) {
        //cout << "REBALANCING" << endl;
        //cout << "x = " << x << endl;
        bool finished = false;
        while (!finished) {
            // we use CHECKPOINT_AND_RUN_QUERY here because rebalancing does not need to be helped if a process is neutralized
            //CHECKPOINT_AND_RUN_QUERY(tid) {
                auto guard = recordmgr->getGuard(tid);
                finished = updateRebalancingStep(tid, key);
            //}
        }
    }
}

// RULE: ANY OUTPUT OF updateXXXXX MUST BE FULLY WRITTEN BEFORE SCX IS INVOKED!
template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::updateInsert(
            const int tid,
            const K& key,
            const V& val,
            const bool onlyIfAbsent,
            V * const result,
            bool * const shouldRebalance) {
    TRACE COUTATOMICTID("insert(tid="<<tid<<", key="<<key<<", val="<<val<<")"<<endl);
    int debugLoopCount = 0;
    Node<K,V> *p = root, *l;
    int count = 0;
    
    // root is never retired, so we don't need to call
    // protect before accessing its child pointers
    l = (Node<K,V>*) root->left.load(memory_order_relaxed);
    
    if (l != NULL) { // there is somenode inside the tree
        while(l != NULL) {
            DEBUG if (++debugLoopCount > 10000) { COUTATOMICTID("tree extremely likely to contain a cycle."<<endl); raise(SIGTERM); }
            if (l->tag != 0 || l->b > 1 || l->b < -1) ++count; // count violations
            p = l;

            assert(p->key != NO_KEY);
            if(cmp(key, p->key)) { // key <= p->key
                l = (Node<K,V>*) p->left.load(memory_order_relaxed);
            } else {
                l = (Node<K,V>*) p->right.load(memory_order_relaxed);
            }
        }
        if (p->key == key) {
            if (onlyIfAbsent) {
                *result = p->value;
                TRACE COUTATOMICTID("return true5\n");
                return true; // success
            }
            
            // replace the value
            p->value = val;
            *result = val;
            TRACE COUTATOMICTID("return true8\n");
            return true;
        } else {
            // Found p->key != key
            auto childl = allocateNode(tid);
            auto childr = allocateNode(tid);
            if (cmp(key, p->key)) {
                // go left
                initializeNode(tid, childl, key, val, 0, 0, NULL, NULL);
                initializeNode(tid, childr, p->key, p->value, 0, 0, NULL, NULL);
                p->value = NO_VALUE;
                p->key = p->key; // DAFAQ
                p->left.store((uintptr_t) childl, memory_order_relaxed);
                p->right.store((uintptr_t) childr, memory_order_relaxed);
                if(p->tag == 1) count--; // count violations
                p->tag -= 1;
                assert(p->tag >= -1);
                p->b = 0;
            } else {
                // go right
                initializeNode(tid, childl, p->key, p->value, 0, 0, NULL, NULL);
                initializeNode(tid, childr, key, val, 0, 0, NULL, NULL);
                p->value = NO_VALUE;
                p->key = key; // DAFAQ
                p->left.store((uintptr_t) childl, memory_order_relaxed);
                p->right.store((uintptr_t) childr, memory_order_relaxed);
                if(p->tag == 1) count--; // count violations
                p->tag -= 1;
                assert(p->tag >= -1);
                p->b = 0;
            }
            *result = val;
            *shouldRebalance = (count > N);
            TRACE COUTATOMIC("count = " << count << endl);
            return true;
        }
    } else {
        TRACE COUTATOMIC("INSERTING FIRST NODE\n");
        // Insert immediately
        auto myNode = allocateNode(tid);
        initializeNode(tid, myNode, key, val, 0, 0, NULL, NULL);
        *result = val;
        p->left.store((uintptr_t) myNode);
        return true;
    }
}

// RULE: ANY OUTPUT OF updateXXXXX MUST BE FULLY WRITTEN BEFORE SCX IS INVOKED!
template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::updateErase(
            const int tid,
            const K& key,
            V * const result,
            bool * const shouldRebalance) {

    TRACE COUTATOMICTID("erase(tid="<<tid<<", key="<<key<<")"<<endl);
    int debugLoopCount = 0;
    Node<K,V> *gp, *p, *l;
    
    gp = NULL;
    p = root;
    l = (Node<K,V>*) root->left.load(memory_order_relaxed);
    
    if (l == NULL) { // Only sentinel in the tree!
        TRACE COUTATOMICTID("return true2\n");
        return true;
    }
    
    int count = 0; // counter for violations found
    
    while((Node<K,V>*) l->left.load(memory_order_relaxed) != NULL) {
        TRACE COUTATOMICTID("traversing tree; l="<<*l<<endl);
        DEBUG if (++debugLoopCount > 10000) { COUTATOMICTID("tree extremely likely to contain a cycle."<<endl); raise(SIGTERM); }

        if (l->tag != 0 || l->b < -1 || l->b > 1) ++count; // count violations on the path
        
        gp = p;
        p = l;
        
        if (cmp(key, p->key)) {
            l = (Node<K,V>*) p->left.load(memory_order_relaxed);
        } else {
            l = (Node<K,V>*) p->right.load(memory_order_relaxed);
        }
    }
    
    // If we fail to find the key in the tree
    if (key != l->key) {
        //TRACE COUTATOMICTID("WHAT THE FUCK");
        *result = NO_VALUE;
        return true; // success;
    } else if (gp == NULL) {
        // single node left in the tree
        p->left.store((uintptr_t) NULL);
        *result = l->value;
        recordmgr->retire(tid, l);
        return true; // success
    } else {
        *result = l->value;

        //TRACE COUTATOMIC("key = " << key << ", NO_KEY = " << NO_KEY << endl);
        assert(key != NO_KEY);

        Node<K,V> *gpleft, *gpright;
        Node<K,V> *pleft, *pright;
        Node<K,V> *sleft, *sright; // sibling
        gpleft= (Node<K,V>*) gp->left.load(memory_order_relaxed);
        gpright = (Node<K,V>*) gp->right.load(memory_order_relaxed);
        pleft= (Node<K,V>*) p->left.load(memory_order_relaxed);
        pright = (Node<K,V>*) p->right.load(memory_order_relaxed);
        
        // Find sibling of l
        Node<K,V> *s = (l == pleft ? pright : pleft);
        sleft = (Node<K,V>*) s->left.load(memory_order_relaxed);
        sright = (Node<K,V>*) s->right.load(memory_order_relaxed);
        
        // Compute new node that replace p (and l)
        // compute new tag
        int newtag = p->tag + s->tag + 1 + ((s == pleft && p->b < 0) | (s == pright && p->b > 0));
        Node<K,V>* myNode = allocateNode(tid);
        initializeNode(tid, myNode, s->key, s->value, newtag, s->b, sleft, sright);
        count += (newtag != 0) - (p->tag != 0 || p->b > 1 || p->b < -1);
        *shouldRebalance = (count > N);
        if(p == gpleft) {
            gp->left.store((uintptr_t)myNode, memory_order_relaxed);
        } else {
            gp->right.store((uintptr_t)myNode, memory_order_relaxed);
        }
        TRACE COUTATOMICTID("return true12\n");
        assert((Node<K,V>*)l->left.load(memory_order_relaxed) == NULL);
        recordmgr->retire(tid, l);
        recordmgr->retire(tid, p);
        return true; // success
    }
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::updateRebalancingStep(
            const int tid,
            const K& key) {
    TRACE COUTATOMICTID("rebalancing(tid="<<tid<<", key="<<key<<")"<<endl);
    Node<K,V> *gp, *p, *l;
    
    l = (Node<K,V>*) root->left.load(memory_order_relaxed);

    //Chromatic_retired_info info;
    //IF_FAIL_TO_PROTECT_NODE(info, tid, l, &root->left, &root->marked) return false; // return and retry
    if (l == NULL) {
        TRACE COUTATOMICTID("Only sentinel in the tree!\n");
        return true; // only sentinel in the tree!
    } else if (l->tag != 0) {
        TRACE COUTATOMICTID("Set the tag of root to 0\n");
        l->tag = 0; // just set the tag of the root to 0
        return true;
    }
    
    gp = p = root;

    while((Node<K,V>*) l->left.load(memory_order_relaxed) != NULL && l->tag == 0 && l->b >= -1 && l->b <= 1) {
        // stop traversing when you found the violation

        gp = p;
        p = l;

        assert(p->key != NO_KEY);
        if (cmp(key, p->key)) {
            l = (Node<K,V>*) p->left.load(memory_order_relaxed);
        } else {
            l = (Node<K,V>*) p->right.load(memory_order_relaxed);
        }
    }
    
    assert(l != NULL);
    if (l->tag == 0 && l->b >= -1 && l->b <= 1) {
        TRACE COUTATOMICTID("No violation found");
        assert((Node<K,V>*) l->left.load(memory_order_relaxed) == NULL);
        return true; // (if no violation found, then we hit a leaf, we can stop)
    }
    
    // a few aliases to make the code more uniform
    // note: these nodes have already been passed to protect() in the
    //       search phase, above. they are currently protected.
    //       thus, we don't need to call protect() before LLXing them.
    // the following variable names follow a specific convention that encodes
    // their ancestry relative to one another.
    //       u is the topmost node.
    //       uX is a child of u, but we don't know/care which child it is.
    //       uXL is the left child of uX. uXR is the right child of uX.
    // more complicated example:
    //       uXXRLR is the right child of uXXRL,
    //       which is the left child of uXXR,
    //       which is the right child of uXX,
    //       which is a child of uX,
    //       which is a child of u.
    Node<K,V> * const u = gp;
    Node<K,V> * const uX = p;
    Node<K,V> * const uXX = l;

    void *iu;       Node<K,V> *uL, *uR;
    void *iuX;      Node<K,V> *uXL, *uXR;
    void *iuXL;      Node<K,V> *uXLL, *uXLR;
    void *iuXR;      Node<K,V> *uXRL, *uXRR;
    void *iuXX;     Node<K,V> *uXXL, *uXXR;
    void *iuXXL;    Node<K,V> *uXXLL, *uXXLR;
    void *iuXXR;    Node<K,V> *uXXRL, *uXXRR;

    //assert(recordmgr->isProtected(tid, u) || u == root);
    //if ((iu = LLX(tid, u, &uL, &uR)) == NULL) return false; // return and retry
    uL = (Node<K,V>*) u->left.load(memory_order_relaxed);
    uR = (Node<K,V>*) u->right.load(memory_order_relaxed);
    bool uXleft = (uX == uL);
    //if (!uXleft && uX != uR) return false; // return and retry

    //assert(recordmgr->isProtected(tid, uX) || uX == root);
    //if ((iuX = LLX(tid, uX, &uXL, &uXR)) == NULL) return false; // return and retry
    uXL = (Node<K,V>*) uX->left.load(memory_order_relaxed);
    uXR = (Node<K,V>*) uX->right.load(memory_order_relaxed);
    bool uXXleft = (uXX == uXL);
    //if (!uXXleft && uXX != uXR) return false; // return and retry

    //assert(recordmgr->isProtected(tid, uXX));
    //if ((iuXX = LLX(tid, uXX, &uXXL, &uXXR)) == NULL) return false; // return and retry
    //uXXL = uXX->left.load(memory_order_relaxed);
    //uXXR = uXX->right.load(memory_order_relaxed);
    //bool uXXXleft = (uXXX == uXXL);
    //if (!uXXXleft && uXXX != uXXR) return false; // return and retry
    
    // any further nodes we LLX will have to have protect calls first
    
    //IF_FAIL_TO_PROTECT_NODE(info, tid, uXXL, &uXX->left, &uXX->marked) return false; // return and retry
    //if ((iuXXL = LLX(tid, uXXL, &uXXLL, &uXXLR)) == NULL) return false; // return and retry
    if(l->tag < 0) {
        /**
            Negative Tag violation (N)
        */
        assert(l->tag == -1);
        if(uXXleft) {
            /**
                Rebalance left negative tag violation
            */
            // Node: You need to LLX additional nodes here
            
            assert(uX->tag == 0);
            if(uX->b <= 0) {
                // Case (n): Moving up negative tag, without any additional modification
                // void *llxResults[] = {...}
                TRACE COUTATOMICTID("Apply Case(n)\n");
                Node<K,V> *nodes[] = {u, uX, uXX};
                return doN(tid, nodes, NULL, uXleft);
            } else {
                // Moving up negative tag causes uX->b become 2
                assert(uX->b == 1);
                if (uXX->b >= 0) {
                    // Case (n)->(n1): single rotation
                    TRACE COUTATOMICTID("Apply Case (n)(n1)\n");
                    Node<K,V> *nodes[] = {u, uX, uXX};
                    return doNN1(tid, nodes, NULL, uXleft);
                } else {
                    // Need help from our right child
                    uXXR = (Node<K,V>*) uXX->right.load(memory_order_relaxed);
                    assert(uXXR != NULL);

                    if(uXXR->tag > 0) {
                        // Case (n)->(n2): moving tags
                        TRACE COUTATOMICTID("Apply Case (n)(n2)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXR};
                        return doNN2(tid, nodes, NULL, uXleft);
                    } else if (uXXR->tag == 0) {
                        // Case (n)->(n3): double rotation
                        TRACE COUTATOMICTID("Apply Case (n)(n3)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXR};
                        return doNN3(tid, nodes, NULL, uXleft);
                    } else if (uXXR->tag == -1) {
                        // Case (n)->(n4): double rotation
                        TRACE COUTATOMICTID("Apply Case (n)(n4)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXR};
                        return doNN4(tid, nodes, NULL, uXleft);
                    }
                }
            }
        } else {
            /**
                Rebalance right negative tag violation
            */
            // n_
            assert(uX->tag == 0);
            assert(uX->tag >= 0);
            
            if (uX->b >= 0) {
                // Case (n_): Moving up negative tag, without any additional modification
                // void *llxResults[] = {...}
                TRACE COUTATOMICTID("Apply Case(n_)\n");
                Node<K,V> *nodes[] = {u, uX, uXX};
                return doN_(tid, nodes, NULL, uXleft);
            } else {
                // Moving negative tag up causes uX->b to be -2
                assert(uX->b == -1);
                if (uXX->b <= 0) {
                    // Case (n_)->(p1): single rotation
                    // void *llxResults[] = {...}
                    TRACE COUTATOMICTID("Apply Case(n_)(p1)\n");
                    Node<K,V> *nodes[] = {u, uX, uXX};
                    return doN_P1(tid, nodes, NULL, uXleft);
                } else {
                    // we need help from our left child
                    uXXL = (Node<K,V>*) uXX->left.load(memory_order_relaxed);
                    assert(uXXL != NULL);

                    if (uXXL->tag > 0) {
                        // Case (n_)->(p2): moving tags
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(n_)(p2)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXL};
                        return doN_P2(tid, nodes, NULL, uXleft);
                    } else if (uXXL->tag == 0) {
                        // Case (n_)->(p3): double rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(n_)(p3)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXL};
                        return doN_P3(tid, nodes, NULL, uXleft);
                    } else if (uXXL->tag < 0) {
                        // Case (n_)->(p4): double rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(n_)(p4)\n");
                        Node<K,V> *nodes[] = {u, uX, uXX, uXXL};
                        return doN_P4(tid, nodes, NULL, uXleft);
                    }
                }
            }
        }
    } else if (l->tag > 0) {
        /**
            Positive Tag Violation (P)
        */
        if(uXXleft) {
            /**
                Rebalance left positive tag violation
            */
            
            if (uX->b >= 0) {
                // Case (p): Moving up positive tag without additional modification
                // void *llxResults[] = {...}
                TRACE COUTATOMICTID("Apply Case(p)\n");
                Node<K,V> *nodes[] = {u, uX, uXX};
                return doP(tid, nodes, NULL, uXleft);
            } else {
                // Applying p causes uX->b to become -2
                assert(uX->b == -1);

                uXR = (Node<K,V>*) uX->right.load(memory_order_relaxed);
                assert(uXR != NULL);
                
                if (uXR->tag > 0) {
                    // Case (p)(p0): move tags
                    // void *llxResults[] = {...}
                    TRACE COUTATOMICTID("Apply Case(p)(p0)\n");
                    Node<K,V> *nodes[] = {u, uX, uXR, uXX};
                    return doPP0(tid, nodes, NULL, uXleft);
                } else if (uXR->b <= 0) {
                    // Case (p)(p1): single rotation
                    // void *llxResults[] = {...}
                    TRACE COUTATOMICTID("Apply Case(p)(p1)\n");
                    Node<K,V> *nodes[] = {u, uX, uXR, uXX};
                    return doPP1(tid, nodes, NULL, uXleft);
                } else {
                    // We need help from the left child of uXR
                    uXRL = (Node<K,V>*) uXR->left.load(memory_order_relaxed);
                    assert(uXRL != NULL);
                    
                    if(uXRL->tag > 0) {
                        // Case (p)(p2): moving tags
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p)(p2)\n");
                        Node<K,V> *nodes[] = {u, uX, uXR, uXRL, uXX};
                        return doPP2(tid, nodes, NULL, uXleft);
                    } else if (uXRL->tag == 0) {
                        // Case (p)(p3): double rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p)(p3)\n");
                        Node<K,V> *nodes[] = {u, uX, uXR, uXRL, uXX};
                        return doPP3(tid, nodes, NULL, uXleft);
                    } else if (uXRL->tag == -1) {
                        // Case (p)(p4): double rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p)(p4)\n");
                        Node<K,V> *nodes[] = {u, uX, uXR, uXRL, uXX};
                        return doPP4(tid, nodes, NULL, uXleft);
                    }
                }
            }
        } else {
            
            // Right child, positive tag
            if (uX->b <= 0) {
                // Case (p_): Moving up positive tag without additional modification
                // void *llxResults[] = {...}
                TRACE COUTATOMICTID("Apply Case(p_)\n");
                Node<K,V> *nodes[] = {u, uX, uXX};
                return doP_(tid, nodes, NULL, uXleft);
            } else {
                assert(uX->b == 1);

                // We need help from our sibling
                uXL = (Node<K,V>*) uX->left.load(memory_order_relaxed);
                assert(uXL != NULL);

                if(uXL->tag > 0) {
                    // Case (p_)(n0): Moving up positive tag
                    // void *llxResults[] = {...}
                    TRACE COUTATOMICTID("Apply Case(p_)(n0)\n");
                    Node<K,V> *nodes[] = {u, uX, uXL, uXX};
                    return doP_N0(tid, nodes, NULL, uXleft);
                } else if(uXL->b >= 0) {
                    // Case (p_)(n1): Single Rotation
                    // void *llxResults[] = {...}
                    TRACE COUTATOMICTID("Apply Case(p_)(n1)\n");
                    Node<K,V> *nodes[] = {u, uX, uXL, uXX};
                    assert((Node<K,V>*) uXL->left.load(memory_order_relaxed) != NULL);
                    return doP_N1(tid, nodes, NULL, uXleft);
                } else {
                    // We need help from the right child of uXL
                    uXLR = (Node<K,V>*) uXL->right.load(memory_order_relaxed);
                    assert(uXLR != NULL);
                    
                    if (uXLR->tag > 0) {
                        // Case (p_)(n2): Moving tags
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p_)(n2)\n");
                        Node<K,V> *nodes[] = {u, uX, uXL, uXLR, uXX};
                        return doP_N2(tid, nodes, NULL, uXleft);
                    } else if (uXLR->tag == 0) {
                        // Case (p_)(n3): Double Rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p_)(n3)\n");
                        Node<K,V> *nodes[] = {u, uX, uXL, uXLR, uXX};
                        return doP_N3(tid, nodes, NULL, uXleft);
                    } else if (uXLR->tag == -1) {
                        // Case (p_)(n4): Double Rotation
                        // void *llxResults[] = {...}
                        TRACE COUTATOMICTID("Apply Case(p_)(n4)\n");
                        Node<K,V> *nodes[] = {u, uX, uXL, uXLR, uXX};
                        return doP_N4(tid, nodes, NULL, uXleft);
                    }
                }
            }
        }
    } 

    COUTATOMICTID("THE IMPOSSIBLE HAPPENED!!!!!"<<endl);
    exit(-1);
}

// rotations
template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, 0, nodes[2]->b, (Node<K,V>*)nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*)nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, nodes[1]->tag-1+(nodes[1]->b < 0), nodes[1]->b+1, nodeUXX, (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUX, memory_order_relaxed);
    } else {
        nodes[0]->right.store((uintptr_t) nodeUX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true; // success
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag + (nodes[1]->b >0);
    int b_u = nodes[1]->b - 1;
    int t_v = nodes[2]->tag - 1;
    int b_v = nodes[2]->b;
    
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, t_v, b_v, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeUXX, (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUX, memory_order_relaxed);
    } else {    
        nodes[0]->right.store((uintptr_t) nodeUX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doNN1(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag-1+(nodes[1]->b < 0); // t_u
    int b_u = nodes[1]->b+1; // b_u
    int t_v = 0;
    int b_v = nodes[2]->b;
    assert(b_u == 2);

    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, 0, 1-(b_v>0), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, t_u+(b_v>0), b_v-1, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), nodeUX);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUXX, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeUXX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doNN2(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag-1+(nodes[1]->b < 0); // t_u
    int b_u = nodes[1]->b+1; // b_u
    int t_v = 0;
    int b_v = nodes[2]->b;
    int t_w = nodes[3]->tag;
    int b_w = nodes[3]->b;
    assert(b_u == 2);
    
    // just moving tags
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_w-1, b_w, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, 0, 0, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), nodeXXR);
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, t_u+1, 1, nodeXX, (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeX, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doNN3(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag-1+(nodes[1]->b < 0); // t_u
    int b_u = nodes[1]->b+1; // b_u
    int t_v = 0;
    int b_v = nodes[2]->b;
    assert(b_u == 2);
    
    // double rotation
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, 0, (nodes[3]->b<0), (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, 0, -(nodes[3]->b>0), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_u+1, 0, nodeXX, nodeX);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doNN4(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag-1+(nodes[1]->b < 0); // t_u
    int b_u = nodes[1]->b+1; // b_u
    int t_v = 0;
    int b_v = nodes[2]->b;
    assert(b_u == 2);

    // double rotation
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, 0, (nodes[3]->b<0)-1, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, 0, 1-(nodes[3]->b>0), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[1]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_u, nodes[3]->b, nodeXX, nodeX);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN_(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, 0, nodes[2]->b, (Node<K,V>*)nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*)nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, nodes[1]->tag-1+(nodes[1]->b > 0), nodes[1]->b-1, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), nodeUXX);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUX, memory_order_relaxed);
    } else {
        nodes[0]->right.store((uintptr_t) nodeUX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true; // success
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN_P1(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag-1+(nodes[1]->b>0); // nodes[1]->t
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = 0; // nodes[2]->t
    int _b_w = nodes[2]->b; // nodes[2]->b

    int t_u = 0;
    int b_u = (_b_w < 0) - 1;
    int t_w = _t_u + (_b_w < 0);
    int b_w = _b_w+1;
    
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, nodeXX, (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN_P2(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag-1+(nodes[1]->b>0); // nodes[1]->t
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = 0; // nodes[2]->t
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag;
    int _b_x = nodes[3]->b;

    int t_u = _t_u + 1;
    int b_u = -1;
    int t_w = 0;
    int b_w = 0;
    int t_x = _t_x - 1;
    int b_x = _b_x;

    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, nodeXXRL, (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXX, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN_P3(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag-1+(nodes[1]->b>0); // nodes[1]->t
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = 0; // nodes[2]->t
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag;
    int _b_x = nodes[3]->b;

    int t_u = 0;
    int b_u = (_b_x < 0);
    int t_w = 0;
    int b_w = -(_b_x > 0);
    int t_x = _t_u+1;
    int b_x = 0;
    
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, nodeXX, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}
    
template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doN_P4(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag-1+(nodes[1]->b>0); // nodes[1]->t
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = 0; // nodes[2]->t
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag;
    int _b_x = nodes[3]->b;

    int t_u = 0;
    int b_u = (_b_x < 0)-1;
    int t_w = 0;
    int b_w = 1-(_b_x > 0);
    int t_x = _t_u;
    int b_x = _b_x;
    

    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, nodeXX, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doPP0(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag+(nodes[1]->b>0); // nodes[1]->tag
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = nodes[2]->tag; // nodes[2]->tag
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_v = nodes[3]->tag-1; // nodes[3]->tag
    int _b_v = nodes[3]->b; // nodes[3]->b

    int t_u = _t_u + 1;
    int b_u = -1;
    int t_w = _t_w - 1;
    int b_w = _b_w;
    int t_v = _t_v;
    int b_v = _b_v;
    
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXL = allocateNode(tid);
    initializeNode(tid, nodeXXL, nodes[3]->key, nodes[3]->value, t_v, b_v, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXXL, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXX, memory_order_relaxed);
    } else {
        nodes[0]->right.store((uintptr_t) nodeXX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doPP1(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag+(nodes[1]->b>0); // nodes[1]->tag
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = nodes[2]->tag; // nodes[2]->tag
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_v = nodes[3]->tag-1; // nodes[3]->tag
    int _b_v = nodes[3]->b; // nodes[3]->b

    int t_u = 0;
    int b_u = (_b_w < 0) - 1;
    int t_w = _t_u + (_b_w < 0);
    int b_w = _b_w+1;
    int t_v = _t_v;
    int b_v = _b_v;

    Node<K,V> *nodeXXL = allocateNode(tid);
    initializeNode(tid, nodeXXL, nodes[3]->key, nodes[3]->value, t_v, b_v, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXXL, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, nodeXX, (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doPP2(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag+(nodes[1]->b>0); // nodes[1]->tag
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = nodes[2]->tag; // nodes[2]->tag
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag; // nodes[3]->tag
    int _b_x = nodes[3]->b; // nodes[3]->b
    int _t_v = nodes[4]->tag-1; // nodes[4]->tag
    int _b_v = nodes[4]->b; // nodes[4]->b

    int t_u = _t_u + 1;
    int b_u = -1;
    int t_w = 0;
    int b_w = 0;
    int t_x = _t_x - 1;
    int b_x = _b_x;
    int t_v = _t_v;
    int b_v = _b_v;
    

    Node<K,V> *nodeXXL = allocateNode(tid);
    initializeNode(tid, nodeXXL, nodes[4]->key, nodes[4]->value, t_v, b_v, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, nodeXXRL, (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXXL, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXX, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doPP3(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag+(nodes[1]->b>0); // nodes[1]->tag
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = nodes[2]->tag; // nodes[2]->tag
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag; // nodes[3]->tag
    int _b_x = nodes[3]->b; // nodes[3]->b
    int _t_v = nodes[4]->tag-1; // nodes[4]->tag
    int _b_v = nodes[4]->b; // nodes[4]->b

    int t_u = 0;
    int b_u = (_b_x < 0);
    int t_w = 0;
    int b_w = -(_b_x > 0);
    int t_x = _t_u+1;
    int b_x = 0;
    int t_v = _t_v;
    int b_v = _b_v;

    Node<K,V> *nodeXXL = allocateNode(tid);
    initializeNode(tid, nodeXXL, nodes[4]->key, nodes[4]->value, t_v, b_v, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXXL, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, nodeXX, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doPP4(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag+(nodes[1]->b>0); // nodes[1]->tag
    int _b_u = nodes[1]->b-1; // nodes[1]->b
    int _t_w = nodes[2]->tag; // nodes[2]->tag
    int _b_w = nodes[2]->b; // nodes[2]->b
    int _t_x = nodes[3]->tag; // nodes[3]->tag
    int _b_x = nodes[3]->b; // nodes[3]->b
    int _t_v = nodes[4]->tag-1; // nodes[4]->tag
    int _b_v = nodes[4]->b; // nodes[4]->b

    int t_u = 0;
    int b_u = (_b_x < 0)-1;
    int t_w = 0;
    int b_w = 1-(_b_x > 0);
    int t_x = _t_u;
    int b_x = _b_x;
    int t_v = _t_v;
    int b_v = _b_v;

    Node<K,V> *nodeXXL = allocateNode(tid);
    initializeNode(tid, nodeXXL, nodes[4]->key, nodes[4]->value, t_v, b_v, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXXL, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[2]->key, nodes[2]->value, t_w, b_w, (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXXRL = allocateNode(tid);
    initializeNode(tid, nodeXXRL, nodes[3]->key, nodes[3]->value, t_x, b_x, nodeXX, nodeXXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXRL, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int t_u = nodes[1]->tag + (nodes[1]->b < 0);
    int b_u = nodes[1]->b + 1;
    int t_v = nodes[2]->tag - 1;
    int b_v = nodes[2]->b;
    
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, t_v, b_v, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[1]->left.load(memory_order_relaxed), nodeUXX);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUX, memory_order_relaxed);
    } else { 
        nodes[0]->right.store((uintptr_t) nodeUX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_N0(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag + (nodes[1]->b<0); // nodes[1]->tag
    int _b_u = nodes[1]->b + 1; // nodes[1]->b
    int _t_v = nodes[2]->tag; // nodes[2]->tag
    int _b_v = nodes[2]->b; // nodes[2]->b
    int _t_w = nodes[3]->tag - 1; // nodes[3]->tag
    int _b_w = nodes[3]->b; // nodes[3]->b
    
    int t_u = _t_u+1;
    int b_u = 1;
    int t_v = _t_v-1;
    int b_v = _b_v;
    int t_w = _t_w;
    int b_w = _b_w;

    Node<K,V> *nodeUXR = allocateNode(tid);
    initializeNode(tid, nodeUXR, nodes[3]->key, nodes[3]->value, t_w, b_w, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed))
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, t_v, b_v, (Node<K,V>*)nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*)nodes[2]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeUXX, nodeUXR);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUX, memory_order_relaxed);
    } else {
        nodes[0]->right.store((uintptr_t) nodeUX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true; // success
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_N1(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag + (nodes[1]->b<0); // nodes[1]->tag
    int _b_u = nodes[1]->b + 1; // nodes[1]->b
    int _t_v = nodes[2]->tag; // nodes[2]->tag
    int _b_v = nodes[2]->b; // nodes[2]->b
    int _t_w = nodes[3]->tag - 1; // nodes[3]->tag
    int _b_w = nodes[3]->b; // nodes[3]->b

    int t_u = 0;
    int b_u = 1-(_b_v>0);
    int t_v = _t_u+(_b_v>0);
    int b_v = _b_v-1;
    int t_w = _t_w;
    int b_w = _b_w;

    Node<K,V> *nodeUXR = allocateNode(tid);
    initializeNode(tid, nodeUXR, nodes[3]->key, nodes[3]->value, t_w, b_w, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed));
    Node<K,V> *nodeUX = allocateNode(tid);
    initializeNode(tid, nodeUX, nodes[1]->key, nodes[1]->value, t_u, b_u, (Node<K,V>*) nodes[2]->right.load(memory_order_relaxed), nodeUXR);
    Node<K,V> *nodeUXX = allocateNode(tid);
    initializeNode(tid, nodeUXX, nodes[2]->key, nodes[2]->value, t_v, b_v, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), nodeUX);

    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeUXX, memory_order_relaxed);
    } else {
        nodes[0]->right.store((uintptr_t) nodeUXX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_N2(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag + (nodes[1]->b<0); // nodes[1]->tag
    int _b_u = nodes[1]->b + 1; // nodes[1]->b
    int _t_v = nodes[2]->tag; // nodes[2]->tag
    int _b_v = nodes[2]->b; // nodes[2]->b
    int _t_w = nodes[3]->tag; // nodes[3]->tag
    int _b_w = nodes[3]->b; // nodes[3]->b
    int _t_x = nodes[4]->tag - 1; // nodes[4]->tag
    int _b_x = nodes[4]->b; // nodes[4]->b
    
    int t_u = _t_u+1;
    int b_u = 1;
    int t_v = 0;
    int b_v = 0;
    int t_w = _t_w-1;
    int b_w = _b_w;
    int t_x = _t_x;
    int b_x = _b_x;
    
    // just moving tags
    Node<K,V> *nodeXR = allocateNode(tid);
    initializeNode(tid, nodeXR, nodes[4]->key, nodes[4]->value, t_x, b_x, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_w, b_w, (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed));
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, t_v, b_v, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), nodeXXR);
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, t_u, b_u, nodeXX, nodeXR);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeX, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeX, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_N3(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag + (nodes[1]->b<0); // nodes[1]->tag
    int _b_u = nodes[1]->b + 1; // nodes[1]->b
    int _t_v = nodes[2]->tag; // nodes[2]->tag
    int _b_v = nodes[2]->b; // nodes[2]->b
    int _t_w = nodes[3]->tag; // nodes[3]->tag
    int _b_w = nodes[3]->b; // nodes[3]->b
    int _t_x = nodes[4]->tag - 1; // nodes[4]->tag
    int _b_x = nodes[4]->b; // nodes[4]->b
    
    int t_u = _t_u;
    int b_u = _b_u;
    int t_v = _t_v;
    int b_v = _b_v;
    int t_w = _t_w;
    int b_w = _b_w;
    int t_x = _t_x;
    int b_x = _b_x;
    
    // double rotation
    Node<K,V> *nodeXR = allocateNode(tid);
    initializeNode(tid, nodeXR, nodes[4]->key, nodes[4]->value, t_x, b_x, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, 0, (_b_w<0), (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, 0, -(_b_w>0), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), nodeXR);
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_u+1, 0, nodeXX, nodeX);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

template<class K, class V, class Compare, class MasterRecordMgr>
bool AVL<K,V,Compare,MasterRecordMgr>::doP_N4(const int tid, Node<K,V> **nodes, void **llxResults, bool fieldIsLeft) {
    int _t_u = nodes[1]->tag + (nodes[1]->b<0); // nodes[1]->tag
    int _b_u = nodes[1]->b + 1; // nodes[1]->b
    int _t_v = nodes[2]->tag; // nodes[2]->tag
    int _b_v = nodes[2]->b; // nodes[2]->b
    int _t_w = nodes[3]->tag; // nodes[3]->tag
    int _b_w = nodes[3]->b; // nodes[3]->b
    int _t_x = nodes[4]->tag - 1; // nodes[4]->tag
    int _b_x = nodes[4]->b; // nodes[4]->b
    
    int t_u = _t_u;
    int b_u = _b_u;
    int t_v = _t_v;
    int b_v = _b_v;
    int t_w = _t_w;
    int b_w = _b_w;
    int t_x = _t_x;
    int b_x = _b_x;

    // double rotation
    Node<K,V> *nodeXR = allocateNode(tid);
    initializeNode(tid, nodeXR, nodes[4]->key, nodes[4]->value, t_x, b_x, (Node<K,V>*) nodes[4]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[4]->right.load(memory_order_relaxed))
    Node<K,V> *nodeXX = allocateNode(tid);
    initializeNode(tid, nodeXX, nodes[2]->key, nodes[2]->value, 0, (b_w<0)-1, (Node<K,V>*) nodes[2]->left.load(memory_order_relaxed), (Node<K,V>*) nodes[3]->left.load(memory_order_relaxed));
    Node<K,V> *nodeX = allocateNode(tid);
    initializeNode(tid, nodeX, nodes[1]->key, nodes[1]->value, 0, 1-(b_w>0), (Node<K,V>*) nodes[3]->right.load(memory_order_relaxed), nodeXR);
    Node<K,V> *nodeXXR = allocateNode(tid);
    initializeNode(tid, nodeXXR, nodes[3]->key, nodes[3]->value, t_u, b_w, nodeXX, nodeX);
    
    if(fieldIsLeft) {
        nodes[0]->left.store((uintptr_t) nodeXXR, memory_order_relaxed);
    } else {
        
        nodes[0]->right.store((uintptr_t) nodeXXR, memory_order_relaxed);
    }
    recordmgr->retire(tid, nodes[1]);
    recordmgr->retire(tid, nodes[2]);
    recordmgr->retire(tid, nodes[3]);
    recordmgr->retire(tid, nodes[4]);
    return true;
}

#endif
