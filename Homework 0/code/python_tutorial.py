#!/usr/bin/env python
import numpy as np

def print_list(l):
    print l

def sort_manual(shops):
    shops_sorted = []
    source = shops.values()
    for i in range(len(source)):
        maxi = max(source[i:]) #find maximum element
        max_index = source[i:].index(maxi) #find index of maximum element
        source[i + max_index] = source[i]  #replace element at max_index with first element
        source[i] = maxi                   #replace first element with max element
        
    #print source
    shops_sorted_notlist = [] #making a variable for temporary. 
    for i in range(len(source)):
        for name, number in shops.iteritems():
            if number == source[i]:
                shops_sorted_notlist += [(name, number)]

    #making the list of list with key and value in the string type as seen in the question paper
    for i in range(len(source)):
        shops_sorted += [((shops_sorted_notlist[i][0]), str(shops_sorted_notlist[i][1]))] 
       
    #print shops
    print 'Manual sorting manual result: '
    print_list(shops_sorted)

def sort_python(shops):
    
    shops_sorted = []
    shops_sorted_notlist = []
    shops_sorted_notlist = sorted(shops.items(), key=lambda x:x[1], reverse = True)

    #making the list of list with key and value in the string type as seen in the question paper
    for i in range(len(shops.values())):
        shops_sorted += [((shops_sorted_notlist[i][0]), str(shops_sorted_notlist[i][1]))]


    print 'Python sorting result: '
    print_list(shops_sorted)

def sort_numpy(shops):
    
    shops_sorted = []
    shops_sorted_notlist = []
    dtype = [('road','S20'),('no', float)]
    values = shops.items()
    a = np.array(values, dtype = dtype)
    a_sort = np.sort(a, order = 'no')
    shops_sorted_notlist  = a_sort[::-1]

    #making the list of list with key and value in the string type as seen in the question paper
    for i in range(len(shops.values())):
        shops_sorted += [((shops_sorted_notlist[i][0]), str(shops_sorted_notlist[i][1]))]
    
    #shops_sorted = np.sort(np.array(shops), order=)

    # TODO: Here implement sorting using numpys build-in sorting function
    
    print 'Numpy sorting result: '
    print_list(shops_sorted)

def main():

    shops = {}
    shops['21st Street'] = 0.9
    shops['Voluto'] = 0.6
    shops['Coffee Tree'] = 0.45
    shops['Tazza D\' Oro'] = 0.75
    shops['Espresso a Mano'] = 0.95
    shops['Crazy Mocha'] = 0.35
    shops['Commonplace'] = 0.5

    sort_manual(shops)
    sort_python(shops)
    sort_numpy(shops)

    

if __name__ == "__main__":
    main()

