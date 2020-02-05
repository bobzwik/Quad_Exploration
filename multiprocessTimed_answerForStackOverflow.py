from multiprocessing import Process, Value
import time

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
            
            # Lower flag to confirm completion of cycle
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

            # Lower flag to confirm completion of cycle
            p1Flag.value = 0


def func2(id, freq, endFlag, p2Flag, runIdx, Ts):
    while (endFlag.value == 0):
        if (p2Flag.value == 1):
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
            
            # Update time for next iteration
            runIdx.value += 1
            # Lower flag to confirm completion of cycle
            p2Flag.value = 0

    

if __name__ == '__main__':
    # Set frequencies of processes
    # Last value of freqs is the fastest one, for process p2
    freqs = [50,100,200]    # Hz
    freqs = [100,200,400]   # Hz
    # freqs = [0.25,0.5,1]  # Hz
    Tf = 10
    Ts = round(1/freqs[-1], 4)
    
    # Create shared values for time index (runIdx)
    # Various flags to trigger the execution of the code in each process (p0Flag, ...)
    # A flag to en all processes
    runIdx = Value('I',0)
    p0Flag = Value('b', 0)
    p1Flag = Value('b', 0)
    p2Flag = Value('b', 0)
    endFlag = Value('b', 0)

    # How many times the fastest process has to run before flagging the slower processes
    p0_counter_exe = freqs[-1]/freqs[0]
    p1_counter_exe = freqs[-1]/freqs[1]

    if (not(freqs[-1] % freqs[0] == 0) or not(freqs[-1] % freqs[1] == 0)):
        raise Exception("Update rates for processes must be a multiple of the dynamic's update rate.")
    if (freqs[-1] < freqs[0]) or (freqs[-1] < freqs[1]):
        raise Exception("Dynamics update rate must be the fastest.")

    # p2 is at fastest frequency, p1 and p0 at lower frequencies
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
        do_p2 = False
        
        # Wait for new timestep AND completion of p2
        if (periodTime >= Ts and p2Flag.value == 0):

            # If p0 or p1 are expected to finish before the new timestep, wait for their completion
            # Depending on the situation, if slower processes have finished their cycle, make do_p2 True
            if (p1_counter == p1_counter_exe) and (p0_counter == p0_counter_exe):
                if (p1Flag.value == 0) and (p0Flag.value == 0):
                    do_p2 = True
            elif (p1_counter == p1_counter_exe):
                if (p1Flag.value == 0):
                    do_p2 = True
            elif (p0_counter == p0_counter_exe):
                if (p0Flag.value == 0):
                    do_p2 = True
            else:
                do_p2 = 1
            
            # If do_p2 is True, raise p2Flag for the p2 process
            if (do_p2):
                periodEnd = time.perf_counter()
                p2Flag.value = 1

                # If it's time to start a cycle for the slower processes, raise their flag and reset their counter
                if (p1_counter == p1_counter_exe):
                    p1Flag.value = 1
                    p1_counter = 0 
                if (p0_counter == p0_counter_exe):
                    p0Flag.value = 1
                    p0_counter = 0
                
                # Increment slower processes counter
                p1_counter += 1
                p0_counter += 1


    # Close all processes
    endFlag.value = 1

    for process in processes:
        process.join()

    print(f'Finished in {round(time.perf_counter()-startTime, 5)} seconds')
    print(Ts*runIdx.value)