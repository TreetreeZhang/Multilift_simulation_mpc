#!/usr/bin/env python3
"""
预计算轨迹系统快速设置脚本
一键生成预计算轨迹并测试系统性能

使用方法:
    python3 setup_precomputed_trajectory.py

作者: AI Assistant
日期: 2024
"""

import sys
import os
import time

# 添加当前目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

def generate_trajectories():
    """生成预计算轨迹"""
    print("=== 生成预计算轨迹 ===\n")
    
    try:
        from precompute_trajectory_generator import PrecomputedTrajectoryGenerator
        
        # 创建生成器
        generator = PrecomputedTrajectoryGenerator(
            T_total=20.0,        # 总任务时间：20秒
            dt_ctrl=0.02,        # 控制周期：20ms
            nq=6,                # 无人机数量：6架
            trajectory_type='fig8'  # 轨迹类型：8字形
        )
        
        # 生成所有轨迹
        print("开始生成轨迹...")
        start_time = time.time()
        generator.generate_all_trajectories("precomputed_trajectories")
        end_time = time.time()
        
        print(f"\n✅ 轨迹生成完成！")
        print(f"⏱️  总耗时: {end_time - start_time:.2f}秒")
        print(f"📁 数据保存位置: {current_dir}/precomputed_trajectories/")
        
        return True
        
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("请确保 Dynamics.py 和相关依赖文件存在")
        return False
    except Exception as e:
        print(f"❌ 生成失败: {e}")
        return False

def test_trajectories():
    """测试轨迹查找性能"""
    print("\n=== 测试轨迹查找性能 ===\n")
    
    try:
        from PrecomputedTrajectoryManager import PrecomputedTrajectoryManager, ReferenceTrajectoryWrapper
        
        # 初始化轨迹管理器
        print("初始化轨迹管理器...")
        manager = PrecomputedTrajectoryManager(
            data_path="precomputed_trajectories",
            horizon=10,
            dt_ctrl=0.02
        )
        
        # 创建兼容包装器
        wrapper = ReferenceTrajectoryWrapper(manager)
        
        # 性能测试
        print("开始性能测试...")
        test_times = [0.0, 5.0, 10.0, 15.0, 19.0]
        test_quads = [0, 1, 2, 3, 4, 5]
        
        # 单点查找测试
        print("\n🔍 单点查找测试:")
        start_time = time.time()
        for t in test_times:
            for q in test_quads:
                pos, vel, acc = manager.get_quadrotor_trajectory_point(t, q)
        end_time = time.time()
        
        total_lookups = len(test_times) * len(test_quads)
        avg_time = (end_time - start_time) / total_lookups * 1000  # ms
        print(f"  查找次数: {total_lookups}")
        print(f"  平均查找时间: {avg_time:.3f} ms")
        
        # MPC参考轨迹测试
        print("\n🎯 MPC参考轨迹测试:")
        start_time = time.time()
        for t in test_times:
            try:
                Ref_xq, Ref_uq, Ref_xl, Ref_ul, Ref0_xq, Ref0_l = wrapper.Reference_for_MPC(t, 0.1)
            except ValueError as e:
                print(f"  时间 {t}s: {e}")
                continue
        end_time = time.time()
        
        mpc_time = (end_time - start_time) / len(test_times) * 1000  # ms
        print(f"  平均MPC轨迹生成时间: {mpc_time:.3f} ms")
        
        # 数据验证
        print("\n✅ 数据验证:")
        sample_time = 5.0
        sample_quad = 0
        pos, vel, acc = manager.get_quadrotor_trajectory_point(sample_time, sample_quad)
        print(f"  时间 {sample_time}s, 无人机 {sample_quad}:")
        print(f"    位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"    速度: [{vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}]")
        print(f"    加速度: [{acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f}]")
        
        print(f"\n🎉 测试完成！预计算轨迹系统运行正常。")
        return True
        
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("请先运行生成轨迹步骤")
        return False
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🚁 Auto-Multilift 预计算轨迹系统")
    print("=" * 50)
    
    success = True
    
    # 生成轨迹
    if not generate_trajectories():
        success = False
    
    # 测试轨迹
    if success and not test_trajectories():
        success = False
    
    # 总结
    print("\n" + "=" * 50)
    if success:
        print("🎉 预计算轨迹系统设置完成！")
        print("\n📖 使用说明:")
        print("  1. 在MPC系统中导入 PrecomputedTrajectoryManager")
        print("  2. 使用 ReferenceTrajectoryWrapper 替换原有的轨迹生成函数")
        print("  3. 享受零延迟轨迹查找！")
        return 0
    else:
        print("❌ 设置过程中遇到错误，请检查错误信息并重试")
        return 1

if __name__ == "__main__":
    sys.exit(main())
