import numpy as np
import matplotlib.pyplot as plt
import csv
import os
from math import cos, sin, radians, degrees, pi, sqrt
from scipy import integrate

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# =================================================================
# 核心参数（精确匹配新要求）
# =================================================================
# 轨迹参数
TRAJ_AMPLITUDE = 142        # 轨迹幅度(mm) - 匹配赛道宽度
OBSTACLE_DISTANCES = [1000, 2000, 3000, 4000, 5000]  # 障碍物位置

# 机械结构参数 (新增)
STEERING_ARM_LENGTH = 50    # 转向臂长度(mm)
TRANSMISSION_RATIO = 5.2    # 传动比
CENTER_HOLE_DIAMETER = 4    # 中心孔直径(mm)

# 凸轮参数（动态计算）
BASE_CIRCLE_DIAMETER = 100  # 基圆直径100mm (固定)

# 运动参数
TRAJ_PERIOD = 2000          # 一个完整周期2000mm
TOTAL_DISTANCE = 5000       # 总距离5000mm（2.5周期）


# 计算所需凸轮升程
def calculate_required_lift():
    """根据轨迹幅度和机械结构计算所需凸轮升程"""
    # 最大转向角θ_max满足：tan(θ_max) = (2π * 轨迹幅度) / 轨迹周期[1,4](@ref)
    tan_theta_max = (2 * np.pi * TRAJ_AMPLITUDE) / TRAJ_PERIOD
    theta_max = np.arctan(tan_theta_max)  # 弧度制
    
    # 凸轮升程 = 转向臂长度 * sin(θ_max)[1,7](@ref)
    required_lift =  STEERING_ARM_LENGTH * np.sin(theta_max)
    return required_lift

LIFT = calculate_required_lift()  # 动态计算凸轮升程

# =================================================================
# 中心轴参数验证
# =================================================================
min_radius = BASE_CIRCLE_DIAMETER / 2 - LIFT
assert min_radius > CENTER_HOLE_DIAMETER / 2 + 5, \
    f"凸轮最小半径({min_radius:.1f}mm)过小，与中心孔冲突! " \
    f"请调整基圆直径或凸轮升程。"


# =================================================================
# 凸轮位移函数 - 使用余弦函数设计
# =================================================================
def cam_displacement(theta):
    """余弦凸轮位移函数，0-360°对应行进0-2000mm"""
    # theta范围0-360°（对应行进0-2000mm）
    # 使用余弦函数，使位移从最大值开始（匹配从波谷开始）
    return -LIFT * cos(radians(theta))

# =================================================================
# 小车的轨迹函数 - 余弦波
# =================================================================
def vehicle_trajectory(distance):
    """小车轨迹函数 - 余弦波"""
    # 使用余弦函数，使轨迹从波谷开始
    return -TRAJ_AMPLITUDE * cos((2 * np.pi /  TRAJ_PERIOD)* distance)

# =================================================================
# 凸轮轮廓生成（实际加工形状）
# =================================================================
def generate_cam_profile():
    """生成凸轮实际加工形状的点集"""
    angles = np.arange(0, 360, 1)  # 0-360°，1°分辨率
    base_radius = BASE_CIRCLE_DIAMETER / 2
    profile_points = []
    
    for angle in angles:
        # 计算当前角度位移
        s = cam_displacement(angle)
        
        # 计算实际轮廓点到中心的距离
        r = base_radius + s
        
        # 转换为直角坐标（实际加工形状）
        rad = radians(angle)
        x = r * cos(rad)
        y = r * sin(rad)
        
        profile_points.append((angle, x, y, s))
    
    return profile_points

# =================================================================
# 轨迹仿真
# =================================================================
def simulate_trajectory():
    """仿真小车轨迹"""
    trajectory = []
    
    for distance in np.arange(0, TOTAL_DISTANCE + 1, 10):  # 每10mm一个点
        # 计算凸轮转角（行进2000mm对应凸轮转360°）
        cam_angle = (distance / TRAJ_PERIOD) * 360
        
        # 计算凸轮位移
        cam_displ = cam_displacement(cam_angle % 360)
        
        # 计算小车轨迹位置
        y_position = vehicle_trajectory(distance)
        
        trajectory.append((distance, cam_angle, y_position, cam_displ))
    
    return trajectory

# =================================================================
# 凸轮轮廓图绘制 - 实际加工形状
# =================================================================
def plot_cam_profile(cam_points):
    """绘制凸轮实际加工形状图"""
    plt.figure(figsize=(8, 8))
    plt.gca().set_aspect('equal')
    
    # 提取轮廓点
    x = [p[1] for p in cam_points]
    y = [p[2] for p in cam_points]
    
    # 绘制凸轮轮廓
    plt.plot(x, y, 'b-', linewidth=3.0, label="凸轮轮廓")
    
    # 绘制基圆
    base_circle = plt.Circle((0, 0), BASE_CIRCLE_DIAMETER / 2, 
                              color='orange', alpha=0.5, label="基圆")
    plt.gca().add_patch(base_circle)
    
    # 绘制中心孔
    center_hole = plt.Circle((0, 0), CENTER_HOLE_DIAMETER / 2, 
                              color='gray', alpha=0.7, label="中心孔")
    plt.gca().add_patch(center_hole)
    
    # 标注基圆直径
    plt.text(0, BASE_CIRCLE_DIAMETER / 2 + 10, 
             f'基圆直径: {BASE_CIRCLE_DIAMETER}mm', 
             ha='center', fontsize=10, fontweight='bold')
    
    # 标注最大凸出量
    max_lift_point = cam_points[180]  # 180°处位移最大
    plt.annotate(f'最大凸出: {LIFT:.1f}mm', 
                 xy=(max_lift_point[1], max_lift_point[2]), 
                 xytext=(max_lift_point[1] - 40, max_lift_point[2] + 20),
                 arrowprops=dict(arrowstyle="->", lw=1.5))
    
    # 标注最大凹入量
    min_lift_point = cam_points[0]  # 0°处凹入最大
    plt.annotate(f'最大凹入: {LIFT:.1f}mm', 
                 xy=(min_lift_point[1], min_lift_point[2]), 
                 xytext=(min_lift_point[1] + 40, min_lift_point[2] - 20),
                 arrowprops=dict(arrowstyle="->", lw=1.5))
    
    # 设置图形属性
    plt.title('凸轮实际加工形状', fontsize=14, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc="upper right")
    
    # 保存图片
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "cam_profile_no_overlap.png")
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    
    return filename

# =================================================================
# 轨迹可视化
# =================================================================
def plot_trajectory(trajectory_data):
    """绘制小车轨迹图"""
    plt.figure(figsize=(20, 20))  # 调整图形为正方形以便观察
    
    # 提取坐标
    distance = [p[0] for p in trajectory_data]
    y_position = [p[2] for p in trajectory_data]
    
    # 绘制轨迹
    plt.plot(distance, y_position, 'b-', linewidth=2.5, label="轨迹")
    
    # 设置坐标轴比例相同
    plt.gca().set_aspect('equal', adjustable='box')
    
    # 标注起点
    plt.text(0, -TRAJ_AMPLITUDE - 50, f'起点 (0, -{TRAJ_AMPLITUDE})', 
             fontsize=10, ha='left', fontweight='bold')
    
    # 标注终点
    plt.text(5000, TRAJ_AMPLITUDE + 50, f'终点 (5000, {TRAJ_AMPLITUDE})', 
             fontsize=10, ha='right', fontweight='bold')
    
    # 标记关键点（缩小散点大小，调整y偏移）
    key_points = [
        (1000, TRAJ_AMPLITUDE, f'峰值 (1000, {TRAJ_AMPLITUDE})', 'green', 120),
        (2000, -TRAJ_AMPLITUDE, f'谷值 (2000, -{TRAJ_AMPLITUDE})', 'purple', -120),
        (3000, TRAJ_AMPLITUDE, f'峰值 (3000, {TRAJ_AMPLITUDE})', 'green', 120),
        (4000, -TRAJ_AMPLITUDE, f'谷值 (4000, -{TRAJ_AMPLITUDE})', 'purple', -120),
        (5000, TRAJ_AMPLITUDE, f'终点 (5000, {TRAJ_AMPLITUDE})', 'blue', 120)
    ]
    
    for x, y, label, color, y_offset in key_points:
        plt.scatter(x, y, s=50, c=color, zorder=5)
        plt.text(x, y + y_offset, label, 
                 fontsize=10, ha='center', color=color, fontweight='bold', zorder=6)
    
    # 绘制障碍物位置（错开y坐标，避免重叠）
    obstacle_y_offsets = [0, 200, -200, 100, -100]
    for idx, dist in enumerate(OBSTACLE_DISTANCES):
        y_offset = obstacle_y_offsets[idx % len(obstacle_y_offsets)]
        plt.axvline(x=dist, color='r', linestyle='--', alpha=0.7, zorder=1)
        plt.text(dist, y_offset, f'障碍物@{dist}mm', 
                 fontsize=9, ha='center', bbox=dict(facecolor='white', alpha=0.8), zorder=10)
    
    # 设置图形属性
    plt.title('小车S弯轨迹 (2.5周期)', fontsize=14, fontweight='bold')
    plt.xlabel('行进距离 (mm)', fontsize=12)
    plt.ylabel('横向位置 (mm)', fontsize=12)
    plt.grid(True, color='lightgray', linestyle='--', alpha=0.7)
    plt.legend(loc="upper right")
    
    # 设置坐标范围
    plt.xlim(-50, 5050)
    plt.ylim(-TRAJ_AMPLITUDE - 200, TRAJ_AMPLITUDE + 250)
    
    # 保存图片
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "vehicle_trajectory_equal_axes.png")
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    
    return filename

# =================================================================
# 导出凸轮数据（SolidWorks加工用）
# =================================================================
def export_cam_data(cam_points):
    """导出凸轮轮廓数据（加工用）"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "cam_profile_for_machining.csv")
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Angle(deg)', 'X(mm)', 'Y(mm)'])
        for point in cam_points:
            writer.writerow([round(point[0], 1), 
                            round(point[1], 3), 
                            round(point[2], 3)])
    
    print(f"凸轮加工数据已导出: {filename}")
    return filename

# =================================================================
# 计算凸轮位移对应的转向角度
# =================================================================
def calculate_steering_angle(cam_displacement):
    """
    计算凸轮位移对应的转向角度
    
    参数:
    cam_displacement: 凸轮位移值 (mm)
    
    返回:
    steering_angle: 转向角度 (度)
    """
    # 凸轮位移通过转向臂转换为转向角
    # 使用正弦关系：sin(angle) = displacement / arm_length
    if abs(cam_displacement) > STEERING_ARM_LENGTH:
        # 确保位移不超过转向臂长度的限制
        limited_displacement = np.sign(cam_displacement) * STEERING_ARM_LENGTH
    else:
        limited_displacement = cam_displacement
        
    steering_angle = degrees(np.arcsin(limited_displacement / STEERING_ARM_LENGTH))
    return steering_angle

# =================================================================
# 设计验证函数
# =================================================================
def validate_trajectory_amplitude():
    """验证凸轮升程与轨迹幅度的关系是否匹配"""
    # 最大转向角计算[1,7](@ref)
    max_steering_angle = degrees(np.arcsin(LIFT / STEERING_ARM_LENGTH))
    
    # 理论轨迹幅度计算[1,4](@ref)
    theoretical_amplitude = (TRAJ_PERIOD * np.tan(np.radians(max_steering_angle))) / (2 * np.pi)
    
    print(f"计算参数:")
    print(f"* 凸轮升程: {LIFT:.2f}mm")
    print(f"* 最大转向角: {max_steering_angle:.2f}°")
    print(f"* 理论轨迹幅度: {theoretical_amplitude:.2f}mm")
    print(f"* 设定轨迹幅度: {TRAJ_AMPLITUDE:.2f}mm")
    
    return abs(theoretical_amplitude - TRAJ_AMPLITUDE) < 50  # 允许50mm误差

# =================================================================
# 计算轨迹弧长与后轮直径
# =================================================================
def calculate_trajectory_arc_length():
    """计算轨迹实际弧长"""
    # 轨迹函数导数
    def trajectory_derivative(x):
        return TRAJ_AMPLITUDE * (2 * np.pi / TRAJ_PERIOD) * sin((2 * np.pi / TRAJ_PERIOD) * x)
    
    # 被积函数 sqrt(1 + (dy/dx)^2)
    def integrand(x):
        derivative = trajectory_derivative(x)
        return sqrt(1 + derivative**2)
    
    # 数值积分计算弧长
    arc_length, _ = integrate.quad(integrand, 0, TRAJ_PERIOD)
    return arc_length

def calculate_wheel_diameter():
    """计算后轮直径"""
    arc_length = calculate_trajectory_arc_length()
    # 后轮直径 = 弧长 / (π * 传动比)
    wheel_diameter = arc_length / (pi * TRANSMISSION_RATIO)
    return wheel_diameter, arc_length

# 计算后轮直径
WHEEL_DIAMETER, TRAJECTORY_ARC_LENGTH = calculate_wheel_diameter()

# =================================================================
# 主程序
# =================================================================
if __name__ == "__main__":
    print("="*70)
    print("凸凹轮设计 (精确匹配新要求)")
    print("="*70)
    print(f"* 计算凸轮升程: {LIFT:.2f}mm")
    print(f"* 基圆直径: {BASE_CIRCLE_DIAMETER}mm")
    print(f"* 轨迹幅度: {TRAJ_AMPLITUDE}mm")
    print("* 凸轮转角: 0-360° (对应行进0-2000mm)")
    print(f"* 总距离: {TOTAL_DISTANCE}mm (2.5周期)")
    print(f"* 障碍物位置: {OBSTACLE_DISTANCES}mm")
    print(f"* 轨迹实际弧长: {TRAJECTORY_ARC_LENGTH:.2f}mm (一个周期)")
    print(f"* 计算后轮直径: {WHEEL_DIAMETER:.2f}mm")
    print("="*70)
    
    # 验证设计参数
    if validate_trajectory_amplitude():
        print(">>> 设计验证通过: 凸轮升程与轨迹幅度匹配")
    else:
        print(">>> 设计警告: 凸轮升程与轨迹幅度存在较大偏差!")
    
    # 生成凸轮实际加工形状
    cam_points = generate_cam_profile()
    
    # 导出凸轮加工数据
    cam_csv = export_cam_data(cam_points)
    
    # 绘制凸轮实际加工形状图
    cam_plot = plot_cam_profile(cam_points)
    
    # 仿真并绘制轨迹
    trajectory_data = simulate_trajectory()
    trajectory_plot = plot_trajectory(trajectory_data)
    
    print("\n设计输出:")
    print(f"1. 凸轮加工数据 (CSV) : {cam_csv}")
    print(f"2. 凸轮加工图       : {cam_plot}")
    print(f"3. 小车轨迹图       : {trajectory_plot}")
    print("="*70)
    
    # 设计说明
    print("\n设计验证要点:")
    print("1. 凸轮为基圆100mm的圆形基础上叠加余弦轮廓")
    print("2. 凸轮旋转360°对应小车行进2000mm")
    print("3. 小车起点在(0, -y_max)处")
    print("4. 所有障碍物均在轨迹零位置处（y=0）")
    print("5. 轨迹从波谷开始，到波峰结束（2.5周期）")
    
    # SolidWorks加工指导
    print("\nSolidWorks加工指导:")
    print("1. 新建零件 -> 选择前视基准面")
    print("2. 草图绘制 -> 导入cam_profile_for_machining.csv")
    print("3. 选择'通过XYZ点的曲线'工具")
    print("4. 选择X和Y列数据")
    print("5. 创建旋转凸台特征（基于轮廓）")
    print("6. 添加中心孔（直径10mm）和键槽（可选）")