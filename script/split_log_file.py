import numpy
import re

def sanitize_filename(name):
    """替换非法字符为下划线"""
    return re.sub(r'[\\/:*?"<>|\s]', '_', name)

def split_log_file(input_file_path):
    # 匹配日志前缀
    log_pattern = re.compile(
        r'^(\d{2}/\d{2}/\d{4} \d{2}:\d{2}:\d{2})\s+\[(\d+)\]\s+(\w+)\s+-\s+([^:]+)'
    )
    
    file_handles = {}
    safe_name = ""
    #with open(input_file_path, 'r', encoding='utf-8') as infile:
    with open(input_file_path, 'r') as infile:
        for line in infile:
            # 跳过空行
            if not line.strip():
                continue
            
            # 匹配日志格式
            match = log_pattern.match(line)
            if not match:
                # 添加到上一个对象
                if len(safe_name) == 0:
                    continue
                # 整行写入
                content =  line
            else:
                # 获取对象名称并处理特殊字符
                object_name = match.group(4)
                safe_name = sanitize_filename(object_name)
                # 去除前缀
                colon_pos = line.find(': ', match.end())
                if colon_pos == -1:  # 格式错误
                    continue
                content = line[colon_pos+2:]
            
            # 获取或创建文件句柄
            if safe_name not in file_handles:
                try:
                    file_handles[safe_name] = open(f"{safe_name}.log", 'w', encoding='utf-8')
                except OSError as e:
                    print(f"无法创建文件 {safe_name}.log: {e}")
                    continue
            
            # 写入日志内容
            file_handles[safe_name].write(content)
    
    # 关闭所有文件句柄
    for fh in file_handles.values():
        fh.close()
        
if __name__ == '__main__':
    split_log_file("ZMotionController.log")

