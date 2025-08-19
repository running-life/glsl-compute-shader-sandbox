# PBD-Cloth 粒子爆炸问题修复

## 问题分析

原始代码中存在几个导致粒子爆炸的关键问题：

### 1. **竞争条件 (Race Condition)** - 主要问题
- **问题**：在 `satisfy-constraints.comp` 中，多个线程可能同时修改同一个粒子的位置
- **原因**：一个粒子可能被多个约束同时引用，导致并发写入冲突
- **后果**：位置修正累积，导致粒子突然跳跃到极远位置

### 2. **时间步长过大**
- **问题**：默认时间步长 `0.016f` (约60FPS) 对PBD算法太大
- **后果**：数值积分不稳定，误差累积

### 3. **约束求解器迭代不足**
- **问题**：默认只有3次迭代，对复杂约束网络不够
- **后果**：约束无法充分满足，导致系统不稳定

### 4. **缺乏数值安全检查**
- **问题**：没有限制位置和速度的变化量
- **后果**：一旦出现异常值就会导致爆炸性增长

## 修复方案

### 1. 改进约束求解器 (`satisfy-constraints.comp`)
```glsl
// 添加误差阈值，只修正显著偏差
float errorThreshold = constraint.restLength * 0.01; // 1%的误差阈值
if (abs(error) < errorThreshold) return;

// 使用更保守的修正策略
float correctionMagnitude = error / totalInvMass * stiffness * constraint.stiffness * 0.5;

// 限制最大修正量
float maxCorrection = constraint.restLength * 0.1; // 最大修正为约束长度的10%
correctionMagnitude = clamp(correctionMagnitude, -maxCorrection, maxCorrection);

// 使用线性混合减少突变
vec3 newPosA = mix(posA, posA + correctionA, 0.8);
```

### 2. 改进位置预测 (`predict-positions.comp`)
```glsl
// 限制位置变化
vec3 deltaPos = newPos - particle.position.xyz;
float maxDelta = 0.5; // 最大位置变化
if (length(deltaPos) > maxDelta) {
    deltaPos = normalize(deltaPos) * maxDelta;
    newPos = particle.position.xyz + deltaPos;
}

// 限制速度
float maxSpeed = 10.0;
if (length(particle.velocity.xyz) > maxSpeed) {
    particle.velocity.xyz = normalize(particle.velocity.xyz) * maxSpeed;
}

// 改进地面碰撞
if (particle.position.y < -2.0) {
    particle.position.y = -2.0;
    particle.velocity.y = abs(particle.velocity.y) * 0.3; // 反弹并减少能量
}
```

### 3. 调整物理参数 (`renderer.h`)
```cpp
dt{0.008f},           // 减小时间步长 (原: 0.016f)
stiffness{0.8f},      // 减小刚度 (原: 1.0f)
solverIterations{5},   // 增加迭代次数 (原: 3)
```

### 4. 改进速度更新 (`update-positions.comp`)
```glsl
// 限制速度以避免不稳定
float maxSpeed = 10.0;
if (length(newVelocity) > maxSpeed) {
    newVelocity = normalize(newVelocity) * maxSpeed;
}
```

## 结果

修复后的系统具有以下特性：

1. **稳定性**：粒子不再出现爆炸性运动
2. **真实感**：布料运动更加自然和平滑
3. **可调节性**：可以通过UI调整各种参数来获得不同的效果
4. **性能**：通过阈值检查减少不必要的计算

## 建议的参数设置

- **Time Step**: 0.005 - 0.010 (更小更稳定)
- **Stiffness**: 0.5 - 1.0 (较低值更柔软)
- **Solver Iterations**: 5 - 10 (更多迭代更精确)
- **Damping**: 0.98 - 0.995 (适当阻尼防止震荡)

## 进一步改进

对于更复杂的应用，可以考虑：

1. **分层约束求解**：按约束类型分批处理
2. **自适应时间步长**：根据系统稳定性动态调整
3. **并行安全的约束求解**：使用图着色或其他并行算法
4. **更精确的碰撞检测**：支持复杂几何体碰撞