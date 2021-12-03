from os import EX_CANTCREAT
from threading import Thread

class Input(Thread):
    def __init__(self):
        self.input = None
        # self.step_amount = 5760 # Counts per rotation of the absolute encoder
        self.step_amount = 1000
        super().__init__(target = self.do_input)
        self.daemon = True
        self.start()
        pass
    def do_input(self):
        try:
            while True:
                self.input = input('w: increase, s: decrease')
        except Exception as e:
            print("Thread error?")
            print(e)

    def reset_input(self):
        self.input = None

    def get_input(self):
        return self.input
    