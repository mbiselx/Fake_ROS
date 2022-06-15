#!/usr/bin/env python

import os
import sys
import getopt

class RosPack():
    """docstring for RosPack"""

    def __init__(self):
        pass

    def get_path(self, pkg, top=os.path.dirname(os.getcwd())) :
        """
        returns the path to a directory with the given name, starting from the parent directory of the cwd
        """
        for root, dirs, files in os.walk(top) :
            if pkg in dirs:
                return os.path.join(root, pkg)

        raise FileNotFoundError("[rospack] Error: package '{}' not found".format(pkg))


if __name__ == "__main__" :
    if len(sys.argv) > 1 :
        for pkg in sys.argv[1:] :
            print("[rospack] Found pkg at : {}".format(RosPack().get_path(pkg)))
    else :
        print("[rospack] No package to find\n\tUsage : python rospack pkg")
