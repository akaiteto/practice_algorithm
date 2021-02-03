# http://www.th.cs.meiji.ac.jp/assets/researches/2005/omoto/heapsort.html
import math

class Tree:
    class Node:
        def __init__(self, data):
            self.element = data
            self.left = None
            self.right = None

    def print(self,N=None):
        if N is not None:
            objtree=N
        else:
            objtree = self.tree
        cnt = 0
        for node in objtree:
            print("idx",cnt," ele:",node.element," left:",node.left, " right:", node.right)
            cnt +=1

    @property
    def max(self):
        return self.tree[0].element

    def change_parent_child(self,parent, child,tree):
        if (child== 2 * parent + 1):
            # left child
            tree[child].element, tree[parent].element = tree[parent].element, tree[child].element
            tree[parent].left = tree[child].element
        elif (child== 2 * parent+2):
            # right child
            tree[child].element, tree[parent].element = tree[parent].element, tree[child].element
            tree[parent].right = tree[child].element
        else:
            assert False ,"想定外エラー"

        # set parent
        if parent != 0:
            if (parent % 2) == 0:
                parent_parent = int(parent / 2) - 1
            else:
                parent_parent = int(parent / 2)
        else :
            return
        if (parent == 2 * parent_parent + 1):
            # left child
            tree[parent_parent].left = tree[parent].element
        elif (parent == 2 * parent_parent + 2):
            # right child
            tree[parent_parent].right = tree[parent].element
        else:
            assert False, "想定外エラー"

    def __init__(self, arr):
        self.tree = self.ArraytoTree(arr)

    def ArraytoTree(self, arr):
        N = []
        for idx in range(len(arr)):
            tmpNode = self.Node(arr[idx])
            N.append(tmpNode)
        for idx in range(len(N)):
            idx_left = 2 * idx + 1
            idx_right = 2 * idx + 2
            # set left-right child
            if 2 * idx + 1 < len(arr):
                N[idx].left = arr[idx_left]
            if 2 * idx + 2 < len(arr):
                N[idx].right = arr[idx_right]

        if len(arr) != 1:
            cnt_ele = len(arr)
            length = cnt_ele - 1
            k = int((length - 1) / 2)
            a = k
            while (a >= 0):
                j = 2 * k + 1
                if j + 1 < len(N):
                    if N[k].element > N[j].element and N[k].element > N[j + 1].element:
                        a = a - 1
                        k = a
                    elif N[j].element > N[k].element and N[j].element > N[j + 1].element:
                        self.change_parent_child(parent=k, child=j, tree=N)
                        k = j
                    elif N[j + 1].element > N[k].element and N[j + 1].element > N[j].element:
                        self.change_parent_child(parent=k, child=j + 1, tree=N)
                        k = j + 1
                else:
                    if N[k].element > N[j].element:
                        a = a - 1
                        k = a
                    elif N[j].element > N[k].element:
                        self.change_parent_child(parent=k, child=j, tree=N)
                        k = j

                if j + 1 < len(N):
                    if (N[j + 1].left == None and N[j + 1].right == None):
                        a = a - 1
                        k = a
                else:
                    if (N[j].left == None and N[j].right == None):
                        a = a - 1
                        k = a
        return N
if __name__ == '__main__':
    lst_org = [4,1,6,2,9,7,3,8]
    tree = Tree(lst_org)
    tree.print()