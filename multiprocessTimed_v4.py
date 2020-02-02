from multiprocessing import Process, Event, Value
from collections import deque
from statistics import mean
import time

# Add Timer class for multiprocessing
class Timer(Process):
    def __init__(self, interval, iteration, function, args=[], kwargs={}, timed=False):
        super(Timer, self).__init__()
        self.interval = interval
        self.iteration = iteration
        self.iterationLeft = self.iteration
        self.timed = timed
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.finished = Event()
        self.correction = 0

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self.finished.set()

    def run(self):
        if self.timed:
            startTimeProcess = time.perf_counter()
            while self.iterationLeft > 0:
                startTimePeriod = time.perf_counter()
                self.function(*self.args, **self.kwargs)
                # print(self.interval-(time.clock() - startTimePeriod))
                self.finished.wait(self.interval-(time.perf_counter() - startTimePeriod) + self.correction)
                self.iterationLeft -= 1
                self.correction = min(0, self.interval-(time.perf_counter() - startTimePeriod))
            print(f'Process finished in {round(time.perf_counter()-startTimeProcess, 5)} seconds')
        else:
            while self.iterationLeft > 0:
                self.function(*self.args, **self.kwargs)
                self.iterationLeft -= 1

class Quadcopter():
    def __init__(self):
        self.pos = 0
    
    def update(self, id, freq, Ts, t, tick_p1, tick_p2):
        # Wait for tick_p2 to have been reset by Process1
        while tick_p2.value == 2:
            pass
        # Wait for tick_p1 to have been reset by Process0
        while tick_p1.value == 4:
            pass
        simTime = Ts*t.value

        # Add fake computational time depending on the frequency of the process
        # print(f'id: {id} at {freq} Hz') 
        if freq == 400:
            time.sleep(0.002)
        elif freq == 200:
            time.sleep(0.003)
        elif freq == 100:
            time.sleep(0.007)
        elif freq == 50:
            time.sleep(0.015)
        
        self.pos += 1

        # Increment tick_p2
        t.value += 1
        tick_p1.value += 1
        tick_p2.value += 1


def func0(id, freq, tick_p1):
    # Wait for 4 runs of Process 2 (tick_p1)
    while tick_p1.value < 4:
        pass
    tick_p1.value = 0   # Reset tick_p1
    
    # Add fake computational time depending on the frequency of the process
    # print(f'id: {id} at {freq} Hz') 
    if freq == 400:
        time.sleep(0.002)
    elif freq == 200:
        time.sleep(0.003)
    elif freq == 100:
        time.sleep(0.007)
    elif freq == 50:
        time.sleep(0.015)


def func1(id, freq, tick_p2):
    # Wait for 2 runs of Process 2 (tick_p2)
    while tick_p2.value < 2:
        pass
    tick_p2.value = 0   # Reset tick_p2

    # Add fake computational time depending on the frequency of the process
    # print(f'id: {id} at {freq} Hz') 
    if freq == 400:
        time.sleep(0.002)
    elif freq == 200:
        time.sleep(0.003)
    elif freq == 100:
        time.sleep(0.007)
    elif freq == 50:
        time.sleep(0.015)


def func2(id, freq, tick_p2):
    # Wait for tick_p2 to have been reset by Process1
    while tick_p2.value >= 2:
        pass
    
    # Add fake computational time depending on the frequency of the process
    # print(f'id: {id} at {freq} Hz') 
    if freq == 400:
        time.sleep(0.002)
    elif freq == 200:
        time.sleep(0.003)
    elif freq == 100:
        time.sleep(0.007)
    elif freq == 50:
        time.sleep(0.015)

    # Increment tick_p2
    tick_p2.value += 1

    

if __name__ == '__main__':
    freqs = [50,100,200]
    # freqs = [0.25,0.5,1]
    Tf = 10
    Ts = 1/freqs[-1]
    t = Value('I',0)

    tick_p1 = Value('i', 1)
    tick_p2 = Value('i', 1)  
    i = 0
    a = 0

    quad = Quadcopter()

    processes = []
    # p0 = Timer(interval=1/freqs[0], iteration=round(Tf*freqs[0]), function = func0, iterativeObject = (i,a), args=(0, freqs[0], tick_p1))
    p0 = Timer(interval=1/freqs[0], iteration=round(Tf*freqs[0]), timed=False, function = func0, args=(0, freqs[0], tick_p1,))
    p1 = Timer(interval=1/freqs[1], iteration=round(Tf*freqs[1]), timed=False, function = func1, args=(1, freqs[1], tick_p2,))
    p2 = Timer(interval=1/freqs[2], iteration=round(Tf*freqs[2]), timed=True, function = quad.update, args=(2, freqs[2], Ts, t, tick_p1, tick_p2))
    processes.append(p0)
    processes.append(p1)
    processes.append(p2)
    
    
    for process in processes:
        process.start()   
    start = time.perf_counter()
    
    for process in processes:
        process.join()
    finish = time.perf_counter()

    print(f'Finished in {round(finish-start, 5)} seconds')
    print(Ts*t.value)
    print(quad.pos)