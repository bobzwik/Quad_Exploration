from multiprocessing import Process, Event, Value
import time

# Add Timer class for multiprocessing
class Timer(Process):
    def __init__(self, interval, iteration, function, iterativeObject, args=[], kwargs={}):
        super(Timer, self).__init__()
        self.interval = interval
        self.iteration = iteration
        self.iterationLeft = self.iteration
        self.iterativeObject = iterativeObject
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.finished = Event()

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self.finished.set()

    def run(self):
        startTimeProcess = time.perf_counter()
        while self.iterationLeft > 0:
            startTimePeriod = time.perf_counter()
            self.iterativeObject = self.function(self.iterativeObject, *self.args, **self.kwargs)
            # print(self.interval-(time.clock() - startTimePeriod))
            self.finished.wait(self.interval-(time.perf_counter() - startTimePeriod))
            self.iterationLeft -= 1
        print(f'Process finished in {round(time.perf_counter()-startTimeProcess, 5)} seconds')
        print(self.iterativeObject)


def func0(iterativeObject, id, freq, tick_p1):
    i = iterativeObject[0]
    a = iterativeObject[1]
    # Wait for 2 runs of Process 1 (tick_p1)
    while tick_p1.value < 2:
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

    i += 1
    a -= 1
    return (i,a)

def func1(i, id, freq, tick_p1, tick_p2):
    # Wait for tick_p1 to have been reset by Process0
    while tick_p1.value >= 2:
        pass
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

    # Increment tick_p1
    tick_p1.value += 1
    i += 1
    return i

def func2(i, id, freq, tick_p2):
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
    i += 1
    return i

    

if __name__ == '__main__':
    freqs = [50,100,200]
    # freqs = [0.25,0.5,1]
    Tf = 10
    
    tick_p1 = Value('i', 1)
    tick_p2 = Value('i', 1)  
    i = 0
    a = 0

    processes = []
    p0 = Timer(interval=1/freqs[0], iteration=round(Tf*freqs[0]), function = func0, iterativeObject = (i,a), args=(0, freqs[0], tick_p1))
    p1 = Timer(interval=1/freqs[1], iteration=round(Tf*freqs[1]), function = func1, iterativeObject = i, args=(1, freqs[1], tick_p1, tick_p2))
    p2 = Timer(interval=1/freqs[2], iteration=round(Tf*freqs[2]), function = func2, iterativeObject = i, args=(2, freqs[2], tick_p2))
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