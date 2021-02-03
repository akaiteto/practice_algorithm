#https://medium-company.com/%E9%81%B8%E6%8A%9E%E3%82%BD%E3%83%BC%E3%83%88/

import copy
import data_struct as dt

def sort_insert(arr):
    """
    Summary:元データから要素を上から抽出して、空いた箱に並べていくだけ。
    """
    def insert_inOrder(arr,element):
        #順番になるようにデータを入れる
        if len(arr)==0:
            arr.append(element)
            return
        for idx in range(len(arr)):
            curr_idx =idx
            if arr[idx] > element:
                arr.insert(curr_idx , element)
                return
            else:
                continue
        arr.insert(len(arr), element)

    arr_output = []
    for idx in range(len(arr)):
        element = arr[idx]
        insert_inOrder(arr_output,element)

    return arr_output

def sort_select(arr):
    """
    Summary:範囲を狭めながら最小値を抽出して並べていく
    """
    def extract_minIdx(arr):
        min_idx = 0
        for idx in range(len(arr)):
            if arr[min_idx] > arr[idx]:
                min_idx = idx
        return min_idx

    arr_target = copy.copy(arr)
    arr_output = []
    for idx_main in range(len(arr_target)):
        min_idx = extract_minIdx(arr_target)
        # 最小値を抽出
        arr_output.append(arr_target[min_idx])
        # 最小値削除 -> 最小値がなくなった配列から再び最小値抽出
        arr_target.pop(min_idx)
    return arr_output

def sort_bubble(arr):
    """
    Summary:大きいものを右端にどんどん追いやっていく
    """
    arr_target = copy.copy(arr)
    for idx_main in range(len(arr_target)):
        for idx_tar in range(len(arr_target)):
            if (idx_tar-1)<0:continue
            if arr_target[idx_tar] < arr_target[idx_tar - 1]:
                arr_target[idx_tar], arr_target[idx_tar - 1] = arr_target[idx_tar - 1], arr_target[idx_tar]
    return arr_target

def sort_shell(arr, interval):
    """
    Summary:グループ分けしながら挿入ソート
    """
    def sort_insert_part(arr, interval):
        arr_group = list(range(0, interval))
        dic_sort_group = {}
        for idx in arr_group:
            dic_sort_group[idx] = []

        for idx_main in arr_group:
            for idx_target in range(idx_main, len(arr), interval):
                dic_sort_group[idx_main].append(arr[idx_target])

        for idx in dic_sort_group.keys():
            dic_sort_group[idx] = sort_insert(dic_sort_group[idx])

        arr_output = [None] * len(arr)

        for idx in range(0, interval):
            cnt = idx
            for item in dic_sort_group[idx]:
                arr_output[cnt] = item
                cnt += interval
        return arr_output

    arr_target = copy.copy(arr)
    interval_tmp = interval
    flgloop = True
    while(flgloop):
        arr_target = sort_insert_part(arr_target, interval_tmp)
        if interval_tmp==1:
            flgloop = False
        interval_tmp = int(interval_tmp / 2)
    return arr_target

def sort_heap(arr):
    """
    Summary:木構造で検出した最大値を抽出していくだけ
    """
    arr_input = copy.copy(arr)
    arr_output = []
    for i in range(len(arr)):
        tree = dt.Tree(arr_input)
        arr_output.append(tree.max)
        idx_max = arr_input.index(tree.max)
        arr_input.pop(idx_max)
    arr_output.reverse()
    return arr_output

def sort_merge(arr):
    """
    Summary:2分割を繰り返したあと、小さい単位でソートしながら結合していく。
    """
    arr_target=copy.copy(arr)

    lst_sep = []
    length = int(len(arr)/2)
    while (True):
        lst_sep.append(length)
        length = int(length/2)
        if length ==1:break
    lst_sep.reverse()
    lst_sep.append(len(arr))

    for step in lst_sep:
        idx_bgn = 0
        idx_fin = idx_bgn + step

        for idx in range(idx_bgn, len(arr_target), step):
            arr_tmp = arr_target[idx_bgn:idx_fin]
            arr_tmp = sort_insert(arr_tmp)
            arr_target[idx_bgn:idx_fin] = arr_tmp
            idx_bgn = idx + step
            idx_fin = idx_bgn + step

    return arr_target

def sort_quick(arr):
    """
    Summary:基準値よりも大きいグループ・小さいグループに分ける操作を繰り返す。
    """
    def devide_BigSmall(arr,idx_Kijun):
        threshold = arr[idx_Kijun]
        big_group = []
        small_group = []
        for idx in range(len(arr)):
            if idx == idx_Kijun:continue
            if threshold > arr[idx]:
                small_group.append(arr[idx])
            else:
                big_group.append(arr[idx])
        idx_Kijun_small = int(len(small_group) / 2)
        if len(small_group)!=0:
            small_group = devide_BigSmall(small_group, idx_Kijun_small)
        if len(big_group)!=0:
            idx_Kijun_big = int(len(big_group) / 2)
            big_group = devide_BigSmall(big_group, idx_Kijun_big)
        return small_group + [threshold] + big_group

    arr_target=copy.copy(arr)
    idx_Kijun = int(len(arr_target)/2)
    arr_output = devide_BigSmall(arr_target,idx_Kijun)

    return arr_output

if __name__ == '__main__':
    lst_org = [17, 11, 12, 5, 14, 9, 6, 16, 4, 10, 1, 19, 13, 15, 0, 2, 3, 18, 7, 8,21]
    print(lst_org,"\r\n")
    lst_sorted_bub = sort_bubble(lst_org)
    lst_sorted_sel = sort_select(lst_org)
    lst_sorted_ins = sort_insert(lst_org)
    lst_sorted_she = sort_shell(lst_org,4)
    lst_sorted_heap = sort_heap(lst_org)
    lst_sorted_merge = sort_merge(lst_org)
    lst_sorted_quick = sort_quick(lst_org)

    print(lst_sorted_bub)
    print(lst_sorted_sel)
    print(lst_sorted_ins)
    print(lst_sorted_she)
    print(lst_sorted_heap,"exist bug")
    print(lst_sorted_merge)
    print(lst_sorted_quick)
