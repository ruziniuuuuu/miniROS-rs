# miniROS-rs 使用指南

## 🚀 快速开始

### 运行 Turtlebot 例子

miniROS-rs 提供了完整的 turtlebot 控制系统示例，包括：

1. **基本控制器** (`12_turtlebot_controller`)
2. **键盘控制** (`13_turtlebot_teleop`) 
3. **物理模拟器** (`14_turtlebot_simulator`)
4. **完整演示** (`15_complete_demo`)

### 运行方式

#### 方法 1：单独运行各个组件

```bash
# 终端 1: 启动模拟器（带可视化）
cargo run --example 14_turtlebot_simulator --features visualization

# 终端 2: 启动键盘控制
cargo run --example 13_turtlebot_teleop
```

#### 方法 2：运行完整演示（推荐）

```bash
# 在单个进程中运行完整系统
cargo run --example 15_complete_demo --features visualization
```

#### 方法 3：运行基本控制器

```bash
# 编程式控制演示
cargo run --example 12_turtlebot_controller
```

## 🔧 常见问题解决

### 端口冲突问题

如果遇到 "Address already in use (os error 48)" 错误：

```bash
# 清理所有 miniROS 进程
pkill -f turtlebot

# 或者清理特定进程
pkill -f 12_turtlebot_controller
pkill -f 13_turtlebot_teleop
pkill -f 14_turtlebot_simulator

# 验证端口是否释放
lsof -i :7400
```

### 键盘控制说明

在键盘控制模式下，需要按键后按 Enter 键：

- `W + Enter`: 前进
- `S + Enter`: 后退
- `A + Enter`: 左转
- `D + Enter`: 右转
- `Q + Enter`: 增加速度
- `E + Enter`: 减少速度
- `Space + Enter`: 紧急停止
- `X + Enter`: 退出

### 可视化功能

启用 Rerun 可视化：

```bash
cargo run --example 14_turtlebot_simulator --features visualization
```

可视化功能包括：
- 🤖 机器人 3D 位置和姿态
- 📈 实时轨迹路径显示
- 📊 速度和位置遥测图表
- 🎯 速度矢量可视化

## 📋 系统架构

```
键盘控制节点 → /cmd_vel 话题 → 模拟器节点 → /odom 话题
                                      ↓
                                 Rerun 可视化
```

## 🔍 调试技巧

### 查看运行进程

```bash
# 查看所有 turtlebot 相关进程
ps aux | grep turtlebot

# 查看端口占用
lsof -i :7400
```

### 日志级别

```bash
# 设置详细日志
RUST_LOG=debug cargo run --example 14_turtlebot_simulator
```

### 编译警告

如果看到编译警告，不影响功能：

```bash
warning: field `node` is never read
```

这是因为某些字段在当前实现中没有直接使用，但保留用于未来扩展。

## 📚 进阶使用

### Python 绑定

```bash
# 构建 Python 绑定
make build-python

# 运行 Python 示例
python python/examples/turtlebot_controller.py
```

### 自定义配置

修改 `src/types.rs` 中的安全限制：

```rust
// 最大线性速度（米/秒）
const MAX_LINEAR_SPEED: f32 = 2.0;

// 最大角速度（弧度/秒）
const MAX_ANGULAR_SPEED: f32 = 4.0;
```

### 跨平台兼容性

- **Linux**: 完整支持所有功能
- **macOS**: 完整支持所有功能  
- **Windows**: 基本功能支持，键盘输入需要按 Enter

## 🎯 最佳实践

1. **避免端口冲突**：一次只运行一个 miniROS 节点
2. **使用完整演示**：推荐使用 `15_complete_demo` 避免进程管理问题
3. **启用可视化**：使用 `--features visualization` 获得最佳体验
4. **正确退出**：使用 `X + Enter` 或 `Ctrl+C` 优雅退出

## 📞 获取帮助

如果遇到问题：

1. 检查进程是否冲突
2. 查看日志输出
3. 参考本指南的故障排除部分
4. 提交 GitHub Issue

## 🔄 更新说明

- **v0.1.4**: 添加 turtlebot 控制系统
- 包含完整的键盘控制、物理模拟和可视化功能
- 支持跨平台运行
- 提供 Python 绑定支持 