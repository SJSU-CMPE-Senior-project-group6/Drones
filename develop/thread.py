import threading
import time

locker = threading.Lock()
locker2 = threading.Lock()

def setup_thread():
    thread1 = threading.Thread(target=callback1)
    thread2 = threading.Thread(target=callback2)
    # thread.join() #need to know how to use join

    list_thread = [
        thread1,
        thread2]
    for thread in list_thread:
        thread.start()

def callback1():
    while True:
        with locker:
        # locker.acquire()
            print("1")
            time.sleep(3)
        #add ur code
        # locker.release()

def callback2():
    while True:
        # locker2.acquire()
        # with locker:
        time.sleep(3)
        print("2")
        #add ur code
        # locker2.release()

if __name__=="__main__":
    setup_thread()
