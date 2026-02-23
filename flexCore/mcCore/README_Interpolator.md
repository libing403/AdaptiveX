# ADX 插补器结构体设计 - 快速参考指南

## 文件导航

| 文件 | 说明 |
|------|------|
| **InterpolatorStructure.h** | 📋 核心结构体定义（直线、圆弧、样条曲线） |
| **InterpolatorDesignGuide.md** | 📚 详细设计文档（数学模型、初始化、执行） |
| **InterpolatorImplementation.c** | 💻 完整实现示例（调用即可使用） |
| **InterpolatorIntegrationGuide.md** | 🔗 集成指南（如何在轴组中使用） |
| **ArcGeometryCalculation.h** | 🎯 圆弧三点法计算（关键算法） |

---

## 一句话总结

| 插补器 | 结构体 | 参数 | 核心公式 |
|--------|--------|------|----------|
| **直线** | `LinearInterpolator_t` | 方向向量 `vec[]`、长度 `length` | $P(s) = P_{start} + s \cdot \vec{u}$ |
| **圆弧** | `ArcInterpolator_t` | 圆心 `center`、半径 `radius`、角度 `theta` | $P(\theta) = \vec{c} + R(\cos\theta \cdot \vec{u}_1 + \sin\theta \cdot \vec{u}_2)$ |
| **样条** | `SplineInterpolator_t` | 控制点 `ctrlPoints`、节点向量 `knot`、参数 `u` | $P(u) = \sum N_{i,p}(u) \cdot C_i$ |

---

## 快速集成（3步）

### Step 1: 在轴组中声明

```c
// groupControl.h 中的 AxisGroupControl_t
typedef struct AxisGroupControl_s {
    UnifiedInterpolator_t inp;              // 统一插补器
    const InterpolatorOps_t *inpOps;        // 函数指针表
} AxisGroupControl_t;
```

### Step 2: 选择插补器类型

```c
// 当收到轨迹指令时
switch (traj->type) {
    case TRAJ_LINE:
        grp->inpOps = &linearInterpolatorOps;
        break;
    case TRAJ_ARC:
        grp->inpOps = &arcInterpolatorOps;
        break;
    case TRAJ_SPLINE:
        grp->inpOps = &splineInterpolatorOps;
        break;
}
```

### Step 3: 执行插补循环

```c
int groupInterpolation(AxisGroupControl_t *grp) {
    if (grp->inp.base.state == 0) {
        grp->inpOps->init(grp, rate);        // 初始化
    }
    grp->inpOps->execute(grp, rate);         // 执行插补
    inverseKinematics(grp);                  // 逆解
    return 0;
}
```

---

## 关键数据流

```
新轨迹指令
    ↓
确定类型 (TRAJ_LINE/ARC/SPLINE)
    ↓
选择函数表 → grp->inpOps = &xxxInterpolatorOps
    ↓
inpOps->init()  ← 计算路径参数（直线方向、圆弧圆心、样条控制点）
    ↓
inpOps->execute() ← 根据速度曲线更新参数（s、θ、u）
    ↓
计算笛卡尔坐标 → grp->inp.base.curPos[]
    ↓
inverseKinematics() ← 转换为关节坐标
    ↓
驱动器
```

---

## 结构体大小对比

| 插补器 | 基础部分 | 特定部分 | 合计 |
|--------|---------|---------|------|
| **直线** | ~200B | ~200B | ~400B |
| **圆弧** | ~200B | ~250B | ~450B |
| **样条** | ~200B | ~动态 | ~500B+ |

💡 使用联合体 `UnifiedInterpolator_t` 时，内存占用为最大值

---

## 执行流程详解

### 直线插补 (Linear)

```
init():
  ✓ 从 CmdTrajectory_t 提取起点、终点
  ✓ 计算方向向量 vec[] (已归一化)
  ✓ 计算长度 length
  ✓ 初始化速度曲线 tp

execute():
  1. 从梯形速度曲线 → 时间 ti
  2. ti → 路径参数 s = trapezoidal_dis(tp, ti)
  3. s → 笛卡尔坐标 P(s) = P_start + s * vec
  4. 计算速度 v = ds/dt
```

**初始化成本：低**  
**执行成本：极低（矢量点乘）**

---

### 圆弧插补 (Arc)

```
init():
  ✓ 输入三点 (P1, P2, P3)
  ✓ 调用 calculateArcFrom3Points()
    → 圆心 center[]
    → 半径 radius
    → 法向量 normal[]
    → 基向量 u1[], u2[]
    → 起始角 theta_start
    → 扫过角 sweepAngle
  ✓ 初始化速度曲线 tp

execute():
  1. 从梯形速度曲线 → 弧长 s
  2. s → 角度 θ = θ_start + s/R
  3. θ → 笛卡尔坐标 P(θ) = center + R*(cos(θ)*u1 + sin(θ)*u2)
  4. 计算速度 v = ds/dt
```

**初始化成本：中等**  
**执行成本：低（三角函数 + 矩阵运算）**

**关键：ArcGeometryCalculation.h 中有完整实现！**

---

### 样条曲线插补 (Spline)

```
init():
  ✓ 关联外部控制点 ctrlPoints
  ✓ 关联节点向量 knotVector
  ✓ 保存次数 degree
  ✓ 预计算弧长表（离线）
  ✓ 初始化速度曲线 tp

execute():
  1. 从梯形速度曲线 → 弧长 s
  2. s → 参数 u = arcLengthToParameter(s)  [查表]
  3. u → 笛卡尔坐标 P(u) = NURBS求值(u)
  4. 计算速度 v = ds/dt
```

**初始化成本：高（需要NURBS库）**  
**执行成本：中等（查表 + 弧长反演）**

**需要集成三方库：opennurbs 或自实现B样条**

---

## 常见问题

### Q1: 直线已经实现了，圆弧需要新增吗？

✅ **是的。** 需要：
1. 复制 `InterpolatorImplementation.c` 中的 `initArcInterpolator()` 和 `executeArcInterpolation()`
2. 集成 `ArcGeometryCalculation.h` 中的 `calculateArcFrom3Points()`
3. 修改 `groupControl.h` 添加 `UnifiedInterpolator_t` 和 `inpOps`

### Q2: 样条曲线如何实现？

⚙️ **需要NURBS库支持**：
1. 选择库：opennurbs（推荐）或自实现
2. 预计算弧长表（离线）
3. 实现 `evaluateBSpline()` 和 `arcLengthToParameter()`
4. 参考 `InterpolatorImplementation.c` 中的框架代码

### Q3: 性能会下降吗？

📊 **不会。** 分析：
- **直线**：现有实现，无变化
- **圆弧**：每周期执行 ~50 次浮点运算（可接受）
- **样条**：与复杂度有关，通常可控

### Q4: 如何处理多轴联动？

🔗 **通过 `inverseKinematics()`**：
1. 插补器计算笛卡尔坐标 `grp->inp.base.curPos[]`
2. 调用运动学逆解 → 关节角
3. 分配给各轴驱动器

---

## 手术式集成清单

- [ ] 复制 `InterpolatorStructure.h` → `flexCore/mcCore/`
- [ ] 复制 `InterpolatorImplementation.c` → `flexCore/mcCore/`
- [ ] 复制 `ArcGeometryCalculation.h` → `flexCore/mcCore/`
- [ ] 修改 `groupControl.h`：添加 `UnifiedInterpolator_t inp;` 和 `InterpolatorOps_t *inpOps;`
- [ ] 修改 `Interpolation.c`：替换现有的 `Inpterpolator_t` 为 `UnifiedInterpolator_t`
- [ ] 修改 `groupInterpolation()` 函数：使用函数指针表调用
- [ ] 测试直线、圆弧、样条曲线插补

---

## 数学参考卡

### 直线
$$\vec{P}(s) = \vec{P}_{start} + s \cdot \hat{u}, \quad s \in [0, L]$$
$$v = \frac{ds}{dt}, \quad a = \frac{dv}{dt}$$

### 圆弧（平面）
$$\vec{P}(\theta) = \vec{c} + R(\cos\theta \cdot \vec{u}_1 + \sin\theta \cdot \vec{u}_2)$$
$$s = R(\theta - \theta_{start})$$
$$v = \frac{ds}{dt}, \quad \omega = \frac{d\theta}{dt} = \frac{v}{R}$$

### 样条曲线（B样条）
$$\vec{P}(u) = \sum_{i=0}^{n} N_{i,p}(u) \cdot \vec{C}_i$$
$$s(u) = \int_0^u \left\| \frac{d\vec{P}}{du'} \right\| du'$$
$$u(s) = \text{反演（查表）}$$

---

## 扩展方向

| 功能 | 难度 | 优先级 |
|------|------|--------|
| 直线插补（已有） | ✅ | P0 |
| 圆弧插补 | 🟡 | P1 |
| 样条曲线插补 | 🔴 | P2 |
| 过渡段（blend） | 🔴 | P2 |
| 前瞻算法 | 🔴 | P3 |
| 自适应精度控制 | 🔴 | P3 |

---

## 相关文件列表

### 核心
- [InterpolatorStructure.h](InterpolatorStructure.h) - 结构体定义
- [InterpolatorImplementation.c](InterpolatorImplementation.c) - 函数实现

### 文档
- [InterpolatorDesignGuide.md](InterpolatorDesignGuide.md) - 详细设计
- [InterpolatorIntegrationGuide.md](InterpolatorIntegrationGuide.md) - 集成指南
- [ArcGeometryCalculation.h](ArcGeometryCalculation.h) - 圆弧算法

### 现有文件
- `Interpolation.c` - 直线插补实现（参考）
- `groupControl.h` - 轴组结构体（待修改）

---

**最后更新：2026-02-19**  
**版本：1.0**

