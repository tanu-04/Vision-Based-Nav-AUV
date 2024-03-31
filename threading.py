import threading
from ultralytics import YOLO
from queue import Queue
import os

# Priority list of class names
priority_classes = ["flare", "Gate", "PIPES", "YELLOW"]

class MyThread(threading.Thread):
    def __init__(self, thread_name, model_path, queue):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.model_path = model_path
        self.queue = queue

    def run(self):
        try:
            model = YOLO(self.model_path)

            results = model.predict(source='/home/tanu/Desktop/yolov8/sauvc', project='/home/tanu/Desktop/yolov8/me/new1', name='test1', exist_ok=True, save_crop=True)

            # Extract detected classes and rearrange according to priority
            detected_classes = []
            for i, result in enumerate(results):
                for j, box in enumerate(result.boxes):
                    class_name = result.names[box.cls[0].item()]
                    if class_name in priority_classes:
                        detected_classes.append(class_name)

            # Write priority classes to a text file if not already present
            file_path = '/home/tanu/Desktop/yolov8/me/priority_classes.txt'
            if not os.path.isfile(file_path):
                open(file_path, 'w').close()  # Create the file if it doesn't exist

            with open(file_path, 'r+') as file:
                existing_classes = [line.strip() for line in file.readlines()]
                for cls in priority_classes:
                    if cls in detected_classes and cls not in existing_classes:
                        file.write(cls + '\n')

            # Notify the queue that this thread has finished
            self.queue.put(self.thread_name)

        except Exception as e:
            print(f"Error occurred in {self.thread_name}: {e}")

def main():
    variables = {
        1: "/home/tanu/Desktop/yolov8/models/flare.pt",
        2: "/home/tanu/Desktop/yolov8/models/gates.pt",
        3: "/home/tanu/Desktop/yolov8/models/pipes.pt"
    }

    queue = Queue()

    user_input = input("Enter numbers of variables to start (e.g., 1 3): ")
    user_variables = [int(var.strip()) for var in user_input.split()]

    threads = []
    for var_num in user_variables:
        if var_num in variables:
            model_path = variables[var_num]
            thread = MyThread(f"Variable {var_num}", model_path, queue)
            thread.start()
            threads.append(thread)
        else:
            print(f"Variable {var_num} not found.")

    # Wait for all initial threads to complete
    for thread in threads:
        thread.join()

    # Start subsequent threads sequentially
    for var_num in range(max(user_variables) + 1, max(variables) + 1):
        if var_num in variables:
            model_path = variables[var_num]
            thread = MyThread(f"Variable {var_num}", model_path, queue)
            thread.start()
            threads.append(thread)

    # Wait for all threads to complete
    for _ in range(len(threads)):
        finished_thread = queue.get()
        print(f"{finished_thread} finished")

if __name__ == "__main__":
    main()
