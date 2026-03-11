import os
import xacro
from lxml import etree

# 定义文件路径
cwd = os.getcwd()
input_filename = os.path.join(cwd, "xMatePro7_4_XML.xacro")
output_filename = os.path.join(cwd, "xmate_pro7.urdf")

print(f"正在处理: {input_filename}...")

# 1. 使用 xacro 库把 Xacro 处理成原始 XML
try:
    doc = xacro.process_file(input_filename)
    xml_string = doc.toprettyxml(indent='  ')
except Exception as e:
    print(f"💥 Xacro 处理出错: {e}")
    exit(1)

# 2. 🔥 核心降维打击：替换掉 package:// 路径
# 我们假设你的 stl 文件就在 meshes 文件夹里，并且meshes与URDF并列
fixed_xml_string = xml_string.replace("package://rokae_description/", "./")

# 3. 保存为最后的 URDF XML 文件
try:
    with open(output_filename, "w") as f:
        f.write(fixed_xml_string)
    print(f"🎉 成功生成标准 XML (URDF): {output_filename}")
except Exception as e:
    print(f"💥 保存文件出错: {e}")
    exit(1)