# Reads in a mapping file which maps ascii codes to hershey vector font character codes.

import re
import sys

def parse(filename):
    """
    Reads in a file that maps ascii codes 32 upwards to hershey vector
    font character codes.

    Returns a list of hershey font character codes, where entry 0 in
    the list corresponds to ASCII code 32, entry 1 is the hershey
    character to use for ASCII character code 33, etc.
    """
    result = []

    for line in open(filename, "r"):
        entries = line.split()
        for entry in entries:
            if re.match('\d+-\d+', entry):
                # Some lines have A-B meaning all values from A to B inclusive
                nums = re.split('-', entry)
                for n in range(int(nums[0]), int(nums[1])+1):
                    result.append(n)
            else:
                result.append(int(entry))

    return result

if __name__ == '__main__':
    result = parse(sys.argv[1])
    print ('parsed map is: {}'.format(result))
