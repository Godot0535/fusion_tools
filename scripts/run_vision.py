import tomlkit
import os
import shutil
import utils

if __name__ == "__main__":
    dataset = "/home/liuyiming/workspace/data/泾河/场景切片真值"
    obj_detect = "/home/liuyiming/workspace/label/jinghe_20240724"
    obj_track = "/home/liuyiming/project/fusion-test/data/jinghe_obj_track"
    lane_detect = "/home/liuyiming/workspace/label/jinghe_results_v2.4"
    lane_track = "/home/liuyiming/workspace/label/jinghe_lane_track_0725"
    obj_ranging = "/home/liuyiming/project/fusion-test/data/jinghe_obj_ranging"
    config_path = (
        "/home/liuyiming/project/fusion-test/apps/vision/config/config.toml"
    )

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
        toml_content["path"]["directory_mode"] = 1
        toml_content["path"]["image"] = dataset + "/" + scene_names[i]
        toml_content["path"]["obj_detect"] = obj_detect + "/" + scene_names[i]
        toml_content["path"]["obj_track"] = obj_track + "/" + scene_names[i]
        toml_content["path"]["lane_detect"] = lane_detect + "/" + scene_names[i]
        toml_content["path"]["lane_track"] = lane_track + "/" + scene_names[i]
        toml_content["path"]["obj_ranging"] = obj_ranging + "/" + scene_names[i]
        toml_content["path"]["mobileye"] = dataset + "/" + scene_names[i]
        with open(config_path, "w", encoding="utf-8") as file:
            file.write(tomlkit.dumps(toml_content))

        os.system(
            "cd /home/liuyiming/project/fusion-test/apps/vision/build && ./vision_app"
        )
