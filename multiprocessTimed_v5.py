from multiprocessing import Process, Value, Pipe, Event
import time


# class Quadcopter():
#     def __init__(self):
#         self.pos = 0
    
#     def update(self, id, freq, Ts, t, tick_p1, tick_p2):
#         # Wait for tick_p2 to have been reset by Process1
#         while tick_p2.value == 2:
#             pass
#         # Wait for tick_p1 to have been reset by Process0
#         while tick_p1.value == 4:
#             pass
#         simTime = Ts*t.value
#         # print(round(simTime,3))
        
#         # Add fake computational time depending on the frequency of the process
#         # print(f'id: {id} at {freq} Hz') 
#         if freq == 400:
#             time.sleep(0.002)
#         elif freq == 200:
#             time.sleep(0.003)
#         elif freq == 100:
#             time.sleep(0.007)
#         elif freq == 50:
#             time.sleep(0.015)
        
#         self.pos += 1


def func0(id, freq, endFlag, p0Flag, runIdx, Ts):
    while (endFlag.value == 0):
        if (p0Flag.value == 1):
            t = round(runIdx.value*Ts, 4)
            # Add fake computational time depending on the frequency of the process
            # print(f'id: {id} at {freq} Hz at {t}s') 
            if freq == 400:
                time.sleep(0.002)
            elif freq == 200:
                time.sleep(0.003)
            elif freq == 100:
                time.sleep(0.007)
            elif freq == 50:
                time.sleep(0.015)
            
            p0Flag.value = 0


def func1(id, freq, endFlag, p1Flag, runIdx, Ts):
    while (endFlag.value == 0):
        if (p1Flag.value == 1):
            t = round(runIdx.value*Ts, 4)

            # Add fake computational time depending on the frequency of the process
            # print(f'id: {id} at {freq} Hz at {t}s') 
            if freq == 400:
                time.sleep(0.002)
            elif freq == 200:
                time.sleep(0.003)
            elif freq == 100:
                time.sleep(0.007)
            elif freq == 50:
                time.sleep(0.015)

            p1Flag.value = 0


def func2(id, freq, endFlag, p2Flag, runIdx, Ts):
    while (endFlag.value == 0):
        if (p2Flag.value == 1):
            t = round(runIdx.value*Ts, 4)

            # Add fake computational time depending on the frequency of the process
            # print(f'id: {id} at {freq} Hz at {t}s') 
            if freq == 500:
                time.sleep(0.0015)
            elif freq == 400:
                time.sleep(0.002)
            elif freq == 200:
                time.sleep(0.003)
            elif freq == 100:
                time.sleep(0.007)
            elif freq == 50:
                time.sleep(0.015)
            
            # Update time for next iteration
            runIdx.value += 1
            p2Flag.value = 0

    

if __name__ == '__main__':
    freqs = [50,100,200]
    freqs = [100,200,400]
    # freqs = [0.25,0.5,1]
    Tf = 10
    Ts = round(1/freqs[-1], 4)
    
    runIdx = Value('I',0)
    endFlag = Value('b', 0)
    p0Flag = Value('b', 0)
    p1Flag = Value('b', 0)
    p2Flag = Value('b', 0)
    p0_counter_exe = freqs[-1]/freqs[0]
    p1_counter_exe = freqs[-1]/freqs[1]

    print(p0_counter_exe)
    print(p1_counter_exe)

    if (not(freqs[-1] % freqs[0] == 0) or not(freqs[-1] % freqs[1] == 0)):
        raise Exception("Update rates for processes must be a multiple of the dynamic's update rate.")
    if (freqs[-1] < freqs[0]) or (freqs[-1] < freqs[1]):
        raise Exception("Dynamics update rate must be the fastest.")

    # Create Pipes
    parent_conn, child_conn = Pipe()


    # p2 is "dynamics" at High frequency, p1 and p0 at lower frequencies
    p0 = Process(target=func0, args=(0, freqs[0], endFlag, p0Flag, runIdx, Ts))
    p1 = Process(target=func1, args=(1, freqs[1], endFlag, p1Flag, runIdx, Ts))
    p2 = Process(target=func2, args=(2, freqs[2], endFlag, p2Flag, runIdx, Ts))
    processes = []
    processes.append(p0)
    processes.append(p1)
    processes.append(p2)

    for process in processes:
        process.start()   
    time.sleep(0.5)
    
    # Start subprocesse's counters to execute directly at the first timestep
    p0_counter = p0_counter_exe
    p1_counter = p1_counter_exe

    # Scheduler
    #------------
    startTime  = time.perf_counter()
    periodEnd = time.perf_counter()
    while (runIdx.value*Ts < Tf):
        periodTime = time.perf_counter()-periodEnd
        do_p2 = 0
        if (periodTime >= Ts and p2Flag.value == 0):     
            periodEnd = time.perf_counter()
            p2Flag.value = 1
            if (p1_counter == p1_counter_exe):
                p1Flag.value = 1
                p1_counter = 0 
            if (p0_counter == p0_counter_exe):
                p0Flag.value = 1
                p0_counter = 0
            p1_counter += 1
            p0_counter += 1
         
         
    endFlag.value = 1

    for process in processes:
        process.join()

    print(f'Finished in {round(time.perf_counter()-startTime, 5)} seconds')
    print(Ts*runIdx.value)