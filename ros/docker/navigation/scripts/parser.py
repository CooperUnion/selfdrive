from concurrent.futures import process
import time
import csv

from waypoint import Waypointers

class CommandParser:
    def __init__(self):
        self.waypointers = Waypointers(single=True, fileRead=False)
        self.read_from_file()

    def read_from_file(self):
        # filename = input('enter filename: ')
        filename = 'cmd1.txt'

        with open(filename, newline='') as file:
            reader = csv.reader(file, delimiter=',', skipinitialspace=True, quotechar='|')
            for row in reader:
                self.process(row)

    def process(self, row): 
        if row[0] == 'stop':
            self.wait(int(row[1]))
        elif row[0] == 'waypoint':
            pass
            result = self.waypointers.send_goal(self.waypointers.create_waypoint(
                row[1], row[2], row[3], row[4], row[5], row[6], row[7]
            ))
            if result == 1:
                print('sent!')
            elif result == -1:
                print('time out!')
        else:
            print(row)
    
    def wait(self, seconds):
        print('sleeping: ', seconds)
        time.sleep(seconds)

    def send_waypoint(self):
        pass

if __name__ == '__main__':
    CommandParser()