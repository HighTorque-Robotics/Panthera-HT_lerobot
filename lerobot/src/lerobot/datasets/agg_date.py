# run_aggregation.py
import logging
from pathlib import Path
from lerobot.datasets.aggregate import aggregate_datasets

# 设置日志级别
logging.basicConfig(level=logging.INFO)

def main():
    """运行数据集整合的例子"""
    # 定义要整合的数据集
    source_datasets = [
        "/home/ht/Panthera-HT/Panthera-HT_lerobot/Panthera-HT_lerobot_dataset/Data Collection Dataset/local/panthera_test_xiadian", 
        "/home/ht/Panthera-HT/Panthera-HT_lerobot/Panthera-HT_lerobot_dataset/Data Collection Dataset/local/panthera_test_4_20_0",  # 替换为你的实际数据集名称
        "/home/ht/Panthera-HT/Panthera-HT_lerobot/Panthera-HT_lerobot_dataset/Data Collection Dataset/local/panthera_test_4_20_1",  # 替换为你的实际数据集名称
    ]
    
    # 定义输出
    output_dataset = "panthera_test_xiadian150"
    output_path = Path("/home/ht/Panthera-HT/Panthera-HT_lerobot/Panthera-HT_lerobot_dataset/Data Collection Dataset/local")  # 输出目录
    
    # 运行整合
    aggregate_datasets(
        repo_ids=source_datasets,
        aggr_repo_id=output_dataset,
        aggr_root=output_path,
        data_files_size_in_mb=500,      # 可选：控制数据文件大小
        video_files_size_in_mb=200,     # 可选：控制视频文件大小
        chunk_size=50                   # 可选：控制文件分块
    )
    print("数据集整合完成！")

if __name__ == "__main__":
    main()
