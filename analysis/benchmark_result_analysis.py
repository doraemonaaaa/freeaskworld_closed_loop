import os
import json
from pathlib import Path
import math
from typing import Any, List, Tuple
import re

# spl_mode = 0, use success rate, =1, use oracle success rate
def calculate_benchmark_results(result_dir, minimum_path_length_list, path_lengths_list, spl_mode=1):
    success_count = 0
    oracle_success_count = 0
    total_count = 0
    total_traj_length = 0
    total_nav_error = 0
    total_oracle_nav_error = 0
    total_askway_num = 0
    spl_sum = 0 
    tl_zero_files = [] 
    
    all_sub_dirs = [d for d in os.listdir(result_dir) if os.path.isdir(os.path.join(result_dir, d))]
    # 按目录名末尾的数字排序
    def extract_number(dir_name):
        match = re.search(r'(\d+)$', dir_name)
        return int(match.group(1)) if match else -1  # 没有数字的目录排在前面
    all_sub_dirs.sort(key=extract_number)

    for sub_dir in all_sub_dirs:
        sub_dir_path = os.path.join(result_dir, sub_dir)
        
        test_metrics_dir = os.path.join(sub_dir_path, "TestMetrics")
        if not os.path.exists(test_metrics_dir) or not os.path.isdir(test_metrics_dir):
            continue
            
        # --- 关键修改 2: 对文件列表也进行排序，确保内部处理顺序一致 ---
        file_names = [f for f in os.listdir(test_metrics_dir) if f.endswith("EpisodeMetrics.json")]
        file_names.sort() 

        for file_name in file_names:
            file_path = os.path.join(test_metrics_dir, file_name)
            
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                if not isinstance(data, dict) or "IsSuccess" not in data or "TrajectoryLength" not in data or "PathLength" not in data:
                    continue
                
                # 安全校验：防止 total_count 超出传入 list 的长度
                if total_count >= len(path_lengths_list):
                    print(f"[警告] 文件数量超过了预期的列表长度，跳过文件: {file_name}")
                    break

                # 核心逻辑：比对数据集路径长度
                dataset_path_len = path_lengths_list[total_count]
                current_path_len = data["PathLength"]

                # 考虑到浮点数精度，可以使用 math.isclose 或设定极小阈值
                if abs(dataset_path_len - current_path_len) > 0.0001:
                    print(f"[错误]: 索引 {total_count} 路径不匹配!")
                    print(f"  文件夹: {sub_dir}")
                    print(f"  Dataset预期: {dataset_path_len}, 当前文件实际: {current_path_len}")
                else:
                    s = 1 if data["IsSuccess"] is True else 0
                    if s == 1:
                        success_count += 1
                    
                    o_s = 1 if data["OracleSuccess"] is True else 0
                    if o_s == 1:
                        oracle_success_count += 1
                    
                    p = float(data["TrajectoryLength"])
                    if p == 0:
                        tl_zero_files.append(file_path)
                    
                    total_traj_length += p
                    
                    if "NavigationError" in data:
                        total_nav_error += float(data["NavigationError"])
                    if "OracleNavigationError" in data:
                        total_oracle_nav_error += float(data["OracleNavigationError"])
                    if "NumberOfDirectionInquiries" in data:
                        total_askway_num += int(data["NumberOfDirectionInquiries"])

                    if spl_mode == 0:
                        l = minimum_path_length_list[total_count]
                        max_lp = max(l, p)
                        if max_lp > 0:  
                            spl_sum += s * (l / max_lp)
                    elif spl_mode == 1:
                        l = minimum_path_length_list[total_count]
                        max_lp = max(l, p)
                        if max_lp > 0:  
                            spl_sum += o_s * (l / max_lp)

                total_count += 1
                    
            except Exception as e:
                print(f"处理文件 {file_path} 时出错: {e}")
    
    # 计算最终指标
    success_rate = (success_count / total_count) * 100 if total_count > 0 else 0
    avg_traj_length = total_traj_length / total_count if total_count > 0 else 0
    spl = spl_sum / total_count if total_count > 0 else 0 
    avg_nav_error = total_nav_error / total_count if total_count > 0 else 0
    avg_oracle_nav_error = total_oracle_nav_error / total_count if total_count > 0 else 0
    oracle_success_rate = (oracle_success_count / total_count) * 100 if total_count > 0 else 0   
    avg_askway_num = total_askway_num / total_count if total_count > 0 else 0
    avg_gtpl = sum(minimum_path_length_list) / total_count if total_count > 0 else 0
    
    return success_rate, avg_traj_length, spl, avg_nav_error, avg_oracle_nav_error, oracle_success_rate, avg_askway_num, tl_zero_files, avg_gtpl

def get_minimum_path_lengths(test_dir):
    total_count = 0
    minimum_path_length_list = []
    path_lengths_list = []
    
    # 获取 test_dir 下所有子目录的列表
    all_sub_dirs = [d for d in os.listdir(test_dir) if os.path.isdir(os.path.join(test_dir, d))]
    all_sub_dirs.sort()
    expected_count = len(all_sub_dirs)
    
    print(f"开始处理Test Dataset，预期子目录总数: {expected_count}")

    # 遍历子目录
    for sub_dir in all_sub_dirs:
        sub_dir_path = os.path.join(test_dir, sub_dir)
        test_metrics_dir = os.path.join(sub_dir_path, "VLNData")
        
        # 检查 VLNData 目录是否存在
        if not os.path.exists(test_metrics_dir):
            print(f"跳过: {sub_dir} 中未找到 VLNData 目录")
            continue
            
        success_in_this_dir = False # 用于标记该子目录下的文件是否成功处理
        
        for file_name in os.listdir(test_metrics_dir):
            if file_name.endswith("VLNData.json"):
                file_path = os.path.join(test_metrics_dir, file_name)
                
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    if not isinstance(data, dict) or "Dialogs" not in data or "GPSDatas" not in data:
                        print(f"错误: {file_name} 数据格式不正确")
                        continue
                    
                    # 1. 提取对话起点信息
                    path_length = data["PathLength"]
                    path_lengths_list.append(path_length)

                    dialog = data["Dialogs"][0]
                    start_pos_dict = dialog["StartPosition"]
                    start_position = [start_pos_dict["x"], start_pos_dict["y"], start_pos_dict["z"]]
                    start_timestamp = dialog["TimeStamp"]
                    end_position = data["EndPosition"]

                    # 2. 在 GPSDatas 中寻找最接近的时间戳
                    gps_datas = data["GPSDatas"]
                    start_index = -1
                    for i, gps in enumerate(gps_datas):
                        if gps["TimeStamp"] >= start_timestamp:
                            start_index = i
                            break
                    threshold = 1.0
                    if start_index != -1:
                        first_gps_pos = gps_datas[start_index]["Position"]
                        diff = math.sqrt(sum((a - b) ** 2 for a, b in zip(start_position, first_gps_pos)))
                        if diff <= threshold:
                            # 计算路径积分：累加后续所有相邻点之间的距离
                            gt_path_dist = 0.0
                            for i in range(start_index, len(gps_datas) - 1):
                                pos_cur = gps_datas[i]["Position"]
                                pos_next = gps_datas[i+1]["Position"]
                                # 计算相邻两点间的距离
                                step_dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pos_cur, pos_next)))
                                gt_path_dist += step_dist
                            
                            minimum_path_length_list.append(gt_path_dist)
                            success_in_this_dir = True
                        else:
                            print(f"验证失败: {sub_dir} 起点偏差过大 ({diff:.4f})")
                except Exception as e:
                    print(f"处理文件 {file_path} 时出错: {e}")

        if success_in_this_dir:
            total_count += 1

    if total_count != expected_count:
        raise ValueError(f"数据处理不完整! 预期子目录数: {expected_count}, 实际成功处理数: {total_count}")
    
    print(f"所有目录处理完毕，共计: {total_count}")
    return minimum_path_length_list, path_lengths_list


if __name__ == "__main__":
    test_dataset_dir = "/home/pengyh/workspace/FreeAskAgent/closed_loop/analysis/HKCity_Test_ClosedLoop"
    result_dir = r"/home/pengyh/workspace/FreeAskAgent/closed_loop/analysis/Benchmarking_vint/Benchmarking20260118_124220"

    (minimum_path_length_list, path_lengths_list) = get_minimum_path_lengths(test_dataset_dir)
    
    if not os.path.exists(result_dir) or not os.path.isdir(result_dir):
        print(f"错误: 指定的目录 '{result_dir}' 不存在或不是一个目录。")
    else:
        sr, tl, spl, ne, one, osr, awn, tl_zero_files, gtpl = calculate_benchmark_results(result_dir, minimum_path_length_list, path_lengths_list, spl_mode=1)
        print(f"SR={sr:.2f}%")
        print(f"GTPL={gtpl:.2f}")
        print(f"TL={tl:.2f}")
        print(f"SPL={spl:.4f}")
        print(f"NE={ne:.2f}")
        print(f"ONE={one:.2f}")
        print(f"OSR={osr:.2f}%")
        print(f"NDI={awn:.2f}")
        if tl_zero_files:
            print(f"\n发现 {len(tl_zero_files)} 个 TrajectoryLength=0 的文件。")
