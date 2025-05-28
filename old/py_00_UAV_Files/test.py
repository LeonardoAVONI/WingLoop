import sys
import time
import logging
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler, FileSystemEvent

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    path = "test"
    #event_handler = LoggingEventHandler()
    is_modified = FileSystemEvent(src_path=path)
    observer = Observer() #Create an instance of the watchdog.observers.Observer thread class.
    observer.schedule(is_modified, path, recursive=False) #Implement a subclass of watchdog.events.FileSystemEventHandler 
    #(or as in our case, we will use the built-in watchdog.events.LoggingEventHandler, which already does).
    # Schedule monitoring a few paths with the observer instance attaching the event handler.
    # Start the observer thread and wait for it generate events without blocking our main thread.
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()