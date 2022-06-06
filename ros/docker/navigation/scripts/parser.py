import time
import csv

from waypoint import Waypointers

class CommandParser:
    def __init__(self):
        self.read_from_file()

    def read_from_file(self):
        # filename = input('enter filename: ')
        filename = 'cmd1.txt'

        with open(filename, newline='') as file:
            reader = csv.reader(file, delimiter=',', skipinitialspace=True, quotechar='|')
            for row in reader:
                print(row)

    def process(self): 
        for command in self.commands:
            lower = command.lower()
            token = lower.split()
            
            if token[0] == 'wait':
                self.wait(int(token[1]))
            elif token[0] == 'waypoint':
                pass
            else:
                print(token)
    
    def wait(self, seconds):
        print('sleeping: ', seconds)
        time.sleep(seconds)

    def send_waypoint(self):
        pass

if __name__ == '__main__':
    CommandParser()