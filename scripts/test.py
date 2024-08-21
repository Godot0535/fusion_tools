import tomlkit

# 读取文件
with open('/home/liuyiming/project/fusion-test/app/vision/config/config.toml', 'r', encoding='utf-8') as file:
    toml_content = tomlkit.parse(file.read())

# 修改内容，确保保留右侧注释
toml_content['display']['switch'] = 0  # 这行代码本身保留了右侧的注释

# 写回文件
with open('/home/liuyiming/project/fusion-test/app/vision/config/config.toml', 'w', encoding='utf-8') as file:
    file.write(tomlkit.dumps(toml_content))
