# http://www.th.cs.meiji.ac.jp/assets/researches/2005/omoto/heapsort.html

class Tree:
    class Node:
        def __init__(self, data):
            self.element = data
            self.left = None
            self.right = None

    def print(self, tree):
        cnt = 0
        for node in tree:
            print("idx",cnt," ele:",node.element, " left:",node.left, " right:", node.right)
            cnt +=1

    def __init__(self, arr):
        N = []
        for idx in range(len(arr)):
            tmpNode = self.Node(arr[idx])
            if 2 * idx + 1 < len(arr):tmpNode.left = arr[2 * idx + 1]
            if 2 * idx + 2 < len(arr):tmpNode.right = arr[2 * idx + 2]
            N.append(tmpNode)

        length = len(arr)-1
        k = int((length )/2)
        a = k
        while (a!=0):
            print("\r\n\r\n\r\n")
            j = 2 * k + 1
            if j + 1 < len(N):
                if N[k].element > N[j].element and N[k].element > N[j + 1].element:
                    a = a - 1
                    k = a
                elif N[j].element > N[k].element and N[j].element > N[j + 1].element:
                    N[j].element, N[k].element = N[k].element, N[j].element
                    k = j
                elif N[j + 1].element > N[k].element and N[j + 1].element > N[j].element:
                    N[j + 1].element, N[k].element = N[k].element, N[j + 1].element
                    k = j + 1
            else:
                if N[k].element > N[j].element:
                    a = a - 1
                    k = a
                elif N[j].element > N[k].element:
                    N[j].element, N[k].element = N[k].element, N[j].element
                    N[j].element, N[k].element = N[k].element, N[j].element

                    self.print(N)
                    exit()
                    k = j


            if j + 1 < len(N):
                if (N[j + 1].left == None and N[j + 1].right == None):
                    a = a - 1
                    k = a
            else:
                if (N[j].left == None and N[j].right == None):
                    a = a - 1
                    k = a
            print("a=", a, " k=", k, " j=", j, " len = ", len(N))
            self.print(N)


if __name__ == '__main__':
    lst_org = [4,1,6,2,9,7,3,8]
    tree = Tree(lst_org)