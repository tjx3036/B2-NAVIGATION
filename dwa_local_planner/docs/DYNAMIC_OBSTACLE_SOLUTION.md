# 动态障碍物膨胀带问题：最优解决方案

## 问题根因

1. **Nav2 默认行为**：obstacle_layer 将**所有** scan 点当作静态障碍物标记
2. **清除机制**：仅靠 raytracing——新 scan 的射线穿过某 cell 才会清除；若机器人不动或传感器未扫到该区域，cell 永不清除
3. **无时间衰减**：`observation_persistence` 只控制消息缓冲时长，不实现 obstacle 的时间衰减
4. **结果**：动态障碍物离开后，原位置及膨胀带长期残留（ghost inflation）

---

## 方案对比

| 方案 | 复杂度 | 效果 | 推荐度 |
|------|--------|------|--------|
| **A. 输入端过滤** | 中 | 高 | ⭐⭐⭐⭐⭐ |
| B. 自定义时间衰减 Layer | 高 | 高 | ⭐⭐⭐ |
| C. 双路 costmap | 高 | 中 | ⭐⭐ |
| D. 强化学习 | 很高 | 不确定 | ⭐ |
| E. 仅调参（DenoiseLayer 等） | 低 | 低 | ⭐ |

---

## 推荐方案：输入端过滤（方案 A）

**核心思路**：在 scan 进入 costmap 之前，过滤掉「确认动态」的障碍物及其刚离开的轨迹，让 costmap 只接收静态/未确认障碍物。

### 数据流

```
/scan ──→ scan_filter_node ──→ /scan_filtered ──→ Nav2 obstacle_layer
              │
              └── ObstaclePredictor（聚类+跟踪+动态判定）
                    └── 过滤：动态质心 + 轨迹带 + 刚离开的 trail
```

### 优势

- **从源头解决**：costmap 不再收到动态障碍物，自然不会产生 ghost
- **已有基础**：`obstacle_predictor_clean.py` 已实现 CV-KF 跟踪、动态/静态判定、`get_confirmed_dynamic_for_costmap_filter()` 等
- **可回退**：只需改 Nav2 的 scan topic，失败时可快速切回 `/scan`
- **不依赖 Nav2 源码**：纯 Python 节点，易维护

### 实现要点

1. **scan_filter_node**：订阅 `/scan`，用 ObstaclePredictor 得到动态区域，剔除对应 scan 点，发布 `/scan_filtered`
2. **Nav2 配置**：obstacle_layer 的 scan topic 改为 `/scan_filtered`
3. **保护静态**：`get_protect_centroids()` 确保静态/未确认区域不被误过滤

---

## 备选：自定义时间衰减 Layer（方案 B）

若输入端过滤效果不足，可增加「时间衰减」：

- 自定义 costmap layer，对每个 obstacle cell 记录最后观测时间
- 超过 `decay_time`（如 1–2 s）未再观测到的 cell 自动清除
- 需 C++ 实现，集成到 Nav2 costmap 插件链

---

## 备选：强化学习（方案 D）

**适用场景**：若传统方法仍无法满足，可尝试 RL。

- **输入**：多帧 scan / costmap + 机器人状态
- **输出**：速度指令
- **隐式学习**：通过时序信息区分动态/静态
- **缺点**：需大量仿真/实车数据、训练成本高、泛化性不确定

**建议**：先完成方案 A，若仍有问题再考虑 RL 作为补充。

---

## 实施步骤（方案 A）

1. 实现 `scan_filter_node.py`：订阅 `/scan`，调用 ObstaclePredictor，发布 `/scan_filtered`
2. 修改 `nav2_params.yaml`：`scan.topic: /scan_filtered`
3. 在 launch 中启动 scan_filter_node
4. 调参：`dynamic_enter_speed`、`dynamic_hold_time`、trail 半径等
