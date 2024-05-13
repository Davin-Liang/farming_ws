#!/usr/bin/env python3
import copy

def modify_list():
    a = [1, 2, [3, 4]]

    modified_list = []
    modified_list.append(a)
    print(modified_list)
    a[0] = 5
    
    modified_list.append(a)
    print(modified_list)

modify_list()