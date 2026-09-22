# v1工具标定迁移记录

v2不再把工具工作位直接定义在理想光心参考坐标系下。所有类别先经过统一的“逆手眼 + 全局修正”得到标准活动TCP，再叠加 `standard_to_tool`。

## 迁移公式

```text
T_base_tcp_inverse = T_base_camera_reference × inverse(T_tcp_camera)
standard_pose = pose(T_base_tcp_inverse) + tcp_correction
T_base_tcp_standard = matrix(standard_pose)
T_standard_tool = inverse(T_base_tcp_standard) × T_base_tcp_taught
```

全局修正只参与标准TCP一次，工具标定和运行阶段不再重复增加。

## 绿色/红色按钮

迁移锚点为 `source/greenbtn_reference_result.json`。最终示教TCP取自v1现场测试记录：

```text
[1086.083347, -217.716642, 626.207499,
 90.724541, -46.955193, 85.669570]
```

迁移后的固定工具偏移为：

```text
xyz_mm  = [106.130627, 56.354203, -16.391591]
rpy_deg = [-3.005278, -4.276208, -0.141311]
```

配置文件保留六位小数。绿色和红色按钮使用同一个物理工具，因此共用该结果。

在v3中，迁移或重新标定后的结果填写到 `config/tool_offsets.yaml`，服务会在每次目标TCP解算时热加载，无需重启。

迁移结果在锚点上精确还原示教TCP。使用第二份历史绿色按钮结果 `source/greenbtn_crosscheck_result.json` 交叉验证时，新旧最终TCP的位置差约 `0.014974 mm`，最大RPY分量差约 `0.000072°`。

## 零偏移类别

`run`、`konb1` 和 `redkonb` 的v1值均为零，并不是有效的相机到活动TCP物理变换。v2将其明确迁移为单位 `standard_to_tool`，含义为：

```text
50mm光心参考 → 逆手眼 → 全局修正 → 最终活动TCP
```

这正是现场验证过的纯光心对齐链。

## 复算命令

```bash
python migrate_legacy_tool_offset.py \
  --legacy-result-json migration/source/greenbtn_reference_result.json \
  --handeye-result calibration/handeye_result.json \
  --tcp-correction=-0.1,11,6.1,1.2,0.3,-6 \
  --taught-tcp=1086.0833474919043,-217.7166416407779,626.2074992400945,90.7245413921842,-46.95519269428417,85.6695698282488 \
  --class-name greenbtn \
  --tool-id tool_green_button
```

完整机器可读记录见 `legacy_tool_migration.json`。迁移只保证坐标数学连续，正式运动前仍需低速验证三个以上目标位置。
