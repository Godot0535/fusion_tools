import os


def get_folders(path):
    """获取路径下的所有文件夹路径并返回

    Args:
        path (str): 指定的路径

    Returns:
        list[str]: 文件夹路径列表
    """
    folders = []
    files = os.listdir(path)
    for file in files:
        path_tmp = path + "/" + file
        if os.path.isdir(path_tmp):
            folders.append(path_tmp)
    folders.sort()
    return folders
