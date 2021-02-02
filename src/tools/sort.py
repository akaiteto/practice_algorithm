#https://medium-company.com/%E9%81%B8%E6%8A%9E%E3%82%BD%E3%83%BC%E3%83%88/

import copy

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

if __name__ == '__main__':
    lst_org = [17, 11, 12, 5, 14, 9, 6, 16, 4, 10, 1, 19, 13, 15, 0, 2, 3, 18, 7, 8,21]
    print(lst_org,"\r\n")
    lst_sorted_bub = sort_bubble(lst_org)
    lst_sorted_sel = sort_select(lst_org)
    lst_sorted_ins = sort_insert(lst_org)
    lst_sorted_she = sort_shell(lst_org,4)

    print(lst_sorted_bub)
    print(lst_sorted_sel)
    print(lst_sorted_ins)
    print(lst_sorted_she)