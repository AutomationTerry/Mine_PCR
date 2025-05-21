import json

# 文件路径配置
input_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled_boundary_xyz.json"
output_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled_boundary_xyz_fine.json"

def process_json_data():
    try:
        # 1. 读取原始JSON文件
        with open(input_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        original_point_count = len(data['points'])
        print(f"原始数据点数: {original_point_count}")
        
        # 2. 处理x<1的点（设为0）
        print("\n[步骤1] 处理 x < 5 的点（设为0）...")
        x_lt1_points = [p for p in data['points'] if p['x'] < 5]
        
        if x_lt1_points:
            print(f"找到 {len(x_lt1_points)} 个 x < 5 的点")
            print("示例点 (前5个):")
            for i, p in enumerate(x_lt1_points[:5], 5):
                print(f"点 {i}: x={p['x']:.6f}, 原slope={p['slope']:.6f}")
            
            # 将这些点的slope设为0
            modified_count = 0
            for p in x_lt1_points:
                if p['slope'] != 0:  # 只统计实际被修改的点
                    modified_count += 1
                p['slope'] = 0
            print(f"已将 {len(x_lt1_points)} 个点的 slope 设为0")
            print(f"(其中 {modified_count} 个点是从非0修改为0)")
        else:
            print("未找到 x < 5 的点")
        
        # 3. 统计处理后的slope范围
        print("\n[步骤2] 统计处理后的 slope 范围...")
        if not data['points']:
            print("警告: 无数据点可供统计")
        else:
            slopes = [p['slope'] for p in data['points']]
            min_slope = min(slopes)
            max_slope = max(slopes)
            slope_zero_count = slopes.count(0)
            non_zero_slopes = [s for s in slopes if s != 0]
            
            print("统计结果:")
            print(f"最小值: {min_slope:.6f}")
            print(f"最大值: {max_slope:.6f}")
            print(f"slope=0 的点数: {slope_zero_count} (占比: {slope_zero_count/len(slopes):.2%})")
            if non_zero_slopes:
                print(f"非零 slope 范围: {min(non_zero_slopes):.6f} ~ {max(non_zero_slopes):.6f}")
                print(f"非零 slope 点数: {len(non_zero_slopes)}")
            else:
                print("所有点的 slope 都为0")
        
        # 4. 保存处理后的数据
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        print(f"\n处理后的数据已保存到: {output_path}")
        
        # 5. 最终统计
        print("\n处理摘要:")
        print(f"原始点数: {original_point_count}")
        print(f"修改 x<1→0 的点数: {len(x_lt1_points)}")
        print(f"最终 slope=0 的点数: {slope_zero_count}")
        
    except FileNotFoundError:
        print(f"错误：文件 {input_path} 未找到！")
    except json.JSONDecodeError:
        print("错误：文件不是有效的JSON格式！")
    except KeyError as e:
        print(f"错误：缺少必要的字段 - {e}")
    except Exception as e:
        print(f"处理过程中发生错误: {e}")

if __name__ == "__main__":
    process_json_data()