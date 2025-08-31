# relaxed_balance_AVL_tree_cpp
C++ implementation of Kim S. Larsen's Relaxed Balance AVL tree [1] on Non-Blocking Tree template [2]

The AVL tree in this code is leaf-oriented. The tree will run rebalancing procedure once it finds a path with enough number of violations.

[1] Larsen, Kim S. "AVL trees with relaxed balance." In Proceedings of 8th International Parallel Processing Symposium, pp. 888-893. IEEE, 1994.

[2] Brown, Trevor, Faith Ellen, and Eric Ruppert. "A general technique for non-blocking trees." In Proceedings of the 19th ACM SIGPLAN symposium on Principles and practice of parallel programming, pp. 329-342. 2014.

## Compiling

```
make main
```

For further customization, please feel free to check out the `Makefile`

## How To Use

Include `AVL.h` in your file, and start declare an object of class `AVL`. 
There are some optional parameters you can adjust on construction:

1. `allowedViolationsPerPath` = the number of violations on the path from the root to the leaf of an update operation before we start balancing the tree
2. `numIterationsPerFixing` = the number of iterations of traversing down the tree from the root to a leaf when trying to fix violations (see the function `fixAllToKey` in `avl_impl.h` for more detail)

The main functions are the following:

1. `V insert(const int tid, const K& key, const V& val)` = insert a pair of (key,val) into the data structure (or simply replace the value if the key already exists)
2. `V insertIfAbsent(const int tid, const K& key, const V& val)` = insert a pair (key,val) into the data structure if the key is not present
3. `pair<V,bool> erase(const int tid, const K& key)` = erase a pair with the key and return a pair of (value, success)
4. `pair<V,bool> find(const int tid, const K& key)` = find a pair with the key and return (value, success)
5. `bool contains(const int tid, const K& key)` = check if the data structure contains the key
6. `int size()` = check the number of leaves in the tree
7. `int height()` = check the height of the tree

