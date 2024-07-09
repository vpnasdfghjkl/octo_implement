import re
# 打开原始文件
with open('scripts/ttttt/command.txt', 'r', encoding='utf-8') as file:
    # 读取所有行
    lines = file.readlines()

# 删除第一行
if lines:
    lines.pop(0)

# 删除每行中的第一个括号及其内部内容
cleaned_lines = []
for line in lines:
   # 使用正则表达式匹配并删除第一个方括号及其内容
    line = re.sub(r'\[.*?\]', '', line, count=1).strip()
    # 删除开头的方括号
    line = re.sub(r'^\[', '', line)
    # 删除结尾的方括号
    line = re.sub(r'\]$', '', line)
    # 将逗号替换为空格
    line = re.sub(r',', ' ', line)
    # 将处理后的行添加到列表中
    cleaned_lines.append(line + '\n')

# 将处理后的内容写入新文件
with open('cleaned.txt', 'w', encoding='utf-8') as file:
    file.writelines(cleaned_lines)

print("文件处理完成。")
