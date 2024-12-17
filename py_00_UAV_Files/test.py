import os

with open('tesss', 'w') as file:
    pass
if os.path.exists("tesss"):
    #current_size = os.path.getsize(file_path)
    current_size = os.stat("tesss").st_size
    print(current_size)