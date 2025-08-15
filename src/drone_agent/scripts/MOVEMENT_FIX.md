# 移动功能修复文档

## 问题描述

在执行多步骤飞行序列时，出现以下错误：

```
➡️ 阶段2: 向前移动 10.0米
----------------------------------------
开始向forward移动 10.0米，速度 2.0m/s
📍 移动起始位置: 高度9.77m, 偏航90.3°
🚁 开始向前移动 10.0米，保持高度9.77m和偏航角90.3°
Offboard移动失败: 'list' object has no attribute 'translate_to_rpc'
```

## 根本原因

**问题根源**: `set_velocity_ned()` 函数需要的是 `VelocityNedYaw` 对象，而不是简单的列表。

**错误代码**:
```python
# 错误的实现
velocity_ned = [speed, 0, 0, current_yaw]  # 这是一个列表
await drone_system.offboard.set_velocity_ned(velocity_ned)
```

**错误原因**: MAVSDK的offboard模式需要特定的数据结构，不能直接使用Python列表。

## 修复方案

### 1. 导入正确的模块
```python
from mavsdk.offboard import VelocityNedYaw
```

### 2. 修复velocity设置
**修复前**:
```python
velocity_ned = [speed, 0, 0, current_yaw]
await drone_system.offboard.set_velocity_ned(velocity_ned)
```

**修复后**:
```python
await drone_system.offboard.set_velocity_ned(
    VelocityNedYaw(speed, 0.0, 0.0, current_yaw)
)
```

### 3. 修复停止移动
**修复前**:
```python
await drone_system.offboard.set_velocity_ned([0, 0, 0, current_yaw])
```

**修复后**:
```python
await drone_system.offboard.set_velocity_ned(
    VelocityNedYaw(0.0, 0.0, 0.0, current_yaw)
)
```

### 4. 添加备用移动方法

为了提高可靠性，添加了备用移动方法：

```python
except Exception as offboard_error:
    print(f"⚠️ Offboard移动失败: {offboard_error}")
    print("🔄 尝试使用备用移动方法...")
    
    try:
        # 备用方法：使用goto_location进行相对移动
        lat_offset = distance * 0.00001 * 1.0
        target_lat = current_lat + lat_offset
        
        await drone_system.action.goto_location(
            target_lat, current_lon, current_alt, current_yaw
        )
        
        await asyncio.sleep(duration + 2)
        return f"✅ 向{direction}移动 {distance}米完成（备用方法）"
        
    except Exception as backup_error:
        return f"❌ 备用移动方法也失败: {backup_error}"
```

## 修复的文件

### `src/drone_agent/scripts/tools/drone.py`

**修改内容**:
1. **导入模块**: 添加 `from mavsdk.offboard import VelocityNedYaw`
2. **修复velocity设置**: 使用正确的VelocityNedYaw对象
3. **添加备用方法**: 当offboard模式失败时的备用移动方案
4. **改善中文提示**: 更清晰的错误信息和状态提示

## VelocityNedYaw 参数说明

```python
VelocityNedYaw(north_m_s, east_m_s, down_m_s, yaw_deg)
```

- **north_m_s**: 北向速度 (m/s) - 正值向北，负值向南
- **east_m_s**: 东向速度 (m/s) - 正值向东，负值向西  
- **down_m_s**: 下向速度 (m/s) - 正值向下，负值向上
- **yaw_deg**: 偏航角 (度) - 无人机朝向

**前进移动示例**:
```python
# 向前移动 2m/s，保持当前偏航角
VelocityNedYaw(2.0, 0.0, 0.0, current_yaw)

# 停止移动，保持当前偏航角
VelocityNedYaw(0.0, 0.0, 0.0, current_yaw)
```

## 测试验证

运行测试脚本验证修复：

```bash
cd src/drone_agent/scripts
python test_movement_fix.py
```

**预期结果**:
- ✅ 无人机成功起飞到指定高度
- ✅ 前进移动不再出现 'translate_to_rpc' 错误
- ✅ 移动过程中保持高度和偏航角稳定
- ✅ 成功完成移动并安全降落

## 修复效果

**修复前**:
```
Offboard移动失败: 'list' object has no attribute 'translate_to_rpc'
```

**修复后**:
```
🎯 启动offboard模式进行前进移动
✅ Offboard模式已启动
📊 移动监控: 高度9.77m (目标9.77m)
🛑 停止移动，恢复位置保持模式
✅ Offboard模式已停止
✅ 向forward移动 10.0米完成
```

## 总结

这次修复解决了以下问题：

1. **✅ 核心错误修复**: 正确使用VelocityNedYaw对象而不是列表
2. **✅ 提高可靠性**: 添加备用移动方法
3. **✅ 改善用户体验**: 更清晰的中文错误提示和状态信息
4. **✅ 保持兼容性**: 修复不影响其他功能

现在多步骤飞行序列中的移动阶段应该能够正常工作，无人机可以成功执行"起飞至10m高度，往前飞10m，然后降落"的完整序列。
