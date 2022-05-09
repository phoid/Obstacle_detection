from re import I
import xml.etree.ElementTree as ET


def parse_xml(file):
    tree = ET.parse(file)
    root = tree.getroot()

    # points is a list of lists of tuples
    # Ex: [[(lat, lon),(lat, lon)], [(lat, lon),(lat, lon)], [(lat, lon),(lat, lon)]]
    points = []
    feature_items = root.findall('FEATUREITEM')
    for item in feature_items:
        raw_points = item.findall('Point2D')
        new_points = []
        for point in raw_points:
            new_points.append((float(point.get('LatDeg')), float(point.get('LonDeg'))))
        points.append(new_points)
    return points

'''
Sorts objects in the XML file based on their latitude coordinates.
'''
def mergeSort(points):
    if len(points) > 1:
        mid = len(points) //2
        left = points[:mid]
        right = points[mid:]

        #recursively call merge sort on each half
        mergeSort(left)
        mergeSort(right)

        #iterators
        i, j, k = 0, 0, 0

        while i < len(left) and j < len(right):
            if left[i][-1][0] <= right[j][-1][0]:
                # Value from the left half has been used
                points[k] = left[i]
                i += 1
            else:
                points[k] = right[j]
                j += 1
            # Move to the next slot
            k += 1

        while i < len(left):
            points[k] = left[i]
            i += 1
            k += 1

        while j < len(right):
            points[k] = right[j]
            j += 1
            k += 1

'''
The two XML files each contain objects that possibly represent the same real-life object
So for each object in the first XML file, we need to run binary search on the second file to see if there are any potential matches

'''
def binary_search(arr, low, high, coords, range):
    # Check base case
    if high >= low:
        mid = (high + low) // 2
        if mid < len(arr):
            # Range is a range chosen by the uer
            # If the elment is present near the middle
            if arr[mid][-1][0] <= coords[0] + range and arr[mid][-1][0] >= coords[0] - range:
                first = mid
                in_range = []

                #go backwards
                try:
                    while arr[mid][-1][0] <= coords[0] + range and arr[mid][-1][0] >= coords[0] - range:
                        in_range.append(arr[mid])
                        mid = mid - 1
                except:
                    pass
                mid = first

                #go forward
                try:
                    while arr[mid][-1][0] <= coords[0] + range and arr[mid][-1][0] >=coords[0] - range:
                        in_range.append(arr[mid])
                        mid = mid + 1
                except:
                    pass
                mid = first
                #check each coordinate in in_range to see if its also within range of the y coordinates
                for pair in in_range:
                    # if y is higher than upper range or lower than bottom range, pop the coordinate off of in_range
                    if pair[-1][1] > coords[1] + range or pair[-1][1] < coords[1] - range:
                        in_range.remove(pair)

                #return the list that contains all corrdinates with x and y coordinates that are in the range
                return in_range

            # If element is smaller than mid, then it can only be present in the left subarray
            elif arr[mid][-1][0] > coords[0]:
                return binary_search(arr, low, mid - 1, coords, range)
            # Else the element can only be present in the right subarray
            else:
                return binary_search(arr, mid + 1, high, coords, range)

def best_match(in_range):
    pass


if __name__ == "__main__":
    points1 = parse_xml(r"./Data/db.xml")
    points2 = parse_xml(r"./Data/db2.xml")

    # Only one needs to be sorted since we're only searching in one
    mergeSort(points2)
    for object in points1:
        in_range = binary_search(points2, 0, len(points2), object, .0005)
        if in_range != [] and in_range:
            match = best_match(in_range)