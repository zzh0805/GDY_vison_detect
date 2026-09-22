# v5.0.0 离线验证记录

环境：Windows，D:/anaconda3/envs/ur_odcam/python.exe。未连接相机、机械臂、MinIO或Ubuntu。

## 完成的验证

- `python -m unittest discover -s tests -q`：102项通过（原88项业务测试＋14项v5相机层测试）。
- `python -m unittest discover -s tool/soft_trigger_test -p test_offline.py -q`：19项通过。
- 新增Python模块compileall语法检查通过。
- 对照本地v3.8，排除.git、缓存、日志、output后，128个既有文件SHA256相同。
  仅5个既有文件变化：README.md、VERSION、handeye_calib/surfacepro50_adapter.py、
  vision_solver/camera.py、tool/soft_trigger_test/bridge.cpp（auto仅在单设备时选择）。
  另增加原生后端、SDK子进程/桥接、构建脚本、测试和v5文档。

配置、模型、标定、业务协议、TCP解算、软件配准、原启动/停止和机械臂脚本未改。

## 新相机层测试内容

使用真实Python子进程作为假SDK端点，测试固定共享缓冲、两次采集的数据独立所有权、
空闲不产生请求、超时杀进程、错误不返回旧缓存、非法尺寸拒绝、关闭阻塞限时返回。
使用合成RGB/Z16验证RGB到BGR、SDK深度尺度乘法、相机内参缩放、原软件配准输出一致性、
无效深度拒绝、纯彩色路径不生成点云、稳定等待不取帧、SDK内存保护、失联适配器清理。
原业务测试覆盖HTTP、快照下载/上传模拟、工具热加载、目标匹配、TCP/进入方向与标定数学。

## 明确未验证

- Linux/aarch64原生桥接编译、链接与厂家SDK实际运行（本机无可用C++编译器/Ubuntu）。
- 真实深度单位、目标范围/点云边缘对齐、彩深同步、完整服务长时间RSS。
- 设备在强制结束SDK子进程后重复连接的可靠性。
- 任何机械臂实际运动，或已有现场TCP的到位精度。

因此本记录是代码离线回归通过，不是无人值守生产稳定性保证。部署前按V5_NATIVE_CAMERA.md
编译并先执行不运动的现场验收。SDK对象析构阻塞采用进程隔离处理，未声称修复厂家问题。
