import tomlkit
import os
import shutil
import utils

if __name__ == "__main__":
    # dataset = "/home/liuyiming/workspace/data/baoji"
    # obj_track_labels = "/home/liuyiming/project/fusion-test/data/obj_track"
    # lane_detect_labels = "/home/liuyiming/workspace/label/baoji_double_lane"
    # lane_track_labels = "/home/liuyiming/workspace/label/baoji_track_lane"
    # obj_ranging_labels = "/home/liuyiming/project/fusion-test/data/obj_ranging"
    # config_path = (
    #     "/home/liuyiming/project/fusion-test/apps/obj_ranging/config/config.toml"
    # )
    dataset = "/home/liuyiming/workspace/data/泾河/场景切片真值"
    obj_track_labels = "/home/liuyiming/project/fusion-test/data/jinghe_obj_track"
    lane_detect_labels = "/home/liuyiming/workspace/label/jinghe_results_v2.4"
    lane_track_labels = "/home/liuyiming/workspace/label/jinghe_lane_track_0725"
    obj_ranging_labels = "/home/liuyiming/project/fusion-test/data/jinghe_obj_ranging"
    config_path = (
        "/home/liuyiming/project/fusion-test/apps/obj_ranging/config/config.toml"
    )

    # 重置输出目录
    if os.path.exists(obj_ranging_labels):
        shutil.rmtree(obj_ranging_labels)
    os.mkdir(obj_ranging_labels)
    
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
        toml_content['path']['obj_track'] = obj_track_labels + "/" + scene_names[i]
        toml_content['path']['lane_detect'] = lane_detect_labels + "/" + scene_names[i]
        toml_content['path']['lane_track'] = lane_track_labels + "/" + scene_names[i]
        toml_content['path']['obj_ranging'] = obj_ranging_labels + "/" + scene_names[i]
        with open(config_path, 'w', encoding='utf-8') as file:
            file.write(tomlkit.dumps(toml_content))
        
        os.mkdir(obj_ranging_labels + "/" + scene_names[i])
        
        os.system("cd /home/liuyiming/project/fusion-test/apps/obj_ranging/build && ./obj_ranging_app")
