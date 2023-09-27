import os
import tkinter
from tkinter import filedialog

def select_folder_from_dialog(prompt):
    """
    Displays a dialog box to select a folder.

    Args:
        prompt (str): Dialog prompt.
    Returns:
        str | None: The selected folder path or None.
    """
    root = __init_tkinter()
    folder = filedialog.askdirectory(parent=root, initialdir=os.getcwd(), title=prompt)

    return folder

def select_files_from_dialog(prompt, allowed_extensions):
    """
    Displays a dialog box to select files.

    Args:
        prompt (str): The prompt message for the dialog box.
        allowed_extensions (list): List of allowed image file extensions.

    Returns:
        list: List of selected image file paths for cropping.
    """
    root = __init_tkinter()
    file_paths = list(filedialog.askopenfilenames(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions]))
    return file_paths

def select_file_from_dialog(prompt, allowed_extensions):
    """
    Displays a dialog box to select files.

    Args:
        prompt (str): The prompt message for the dialog box.
        allowed_extensions (list): List of allowed image file extensions.

    Returns:
        str | None: the path of selected file or None.
    """
    root = __init_tkinter()
    file_path = filedialog.askopenfilename(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions])
    return file_path

def __init_tkinter():
    """
    Initialize Tkinter and make sure the file dialog opens in the foreground.

    Args: None

    Returns:
        root: Tkinter window root.
    """
    root = tkinter.Tk()
    root.withdraw()
    root.wm_attributes('-topmost', 1)
    return root
