import tomlkit
import os
import shutil
import utils

if __name__ == "__main__":
    dataset = "/home/liuyiming/workspace/data/泾河/场景切片真值"
    obj_labels = "/home/liuyiming/workspace/label/jinghe_20240902"
    results = "/home/liuyiming/project/fusion_tools/data/jinghe_obj_track"
    config_path = (
        "/home/liuyiming/project/fusion_tools/apps/obj_track/config/config.toml"
    )

    # 重置输出目录
    if os.path.exists(results):
        shutil.rmtree(results)
    os.mkdir(results)

    # 获取真值中的场景列表
    scene_paths = utils.get_folders(dataset)
    scene_names = []
    for secne_path in scene_paths:
        scene_name = os.path.basename(secne_path)
        if scene_name == "LABELANDMODEL":
            scene_paths.remove(secne_path)
            continue
        scene_names.append(scene_name)

    for i in range(0, len(scene_paths)):
        with open(config_path, "r", encoding="utf-8") as file:
            toml_content = tomlkit.parse(file.read())
        toml_content['path']['directory_mode'] = 1
        toml_content['path']['obj_detect'] = obj_labels + "/" + scene_names[i]
        toml_content['path']['obj_track'] = results + "/" + scene_names[i]
        with open(config_path, 'w', encoding='utf-8') as file:
            file.write(tomlkit.dumps(toml_content))
        
        os.mkdir(results + "/" + scene_names[i])
        
        os.system("cd /home/liuyiming/project/fusion_tools/apps/obj_track/build && ./obj_track_app")
        