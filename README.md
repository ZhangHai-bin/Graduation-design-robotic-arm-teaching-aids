# 🤖 空间三自由度机械臂教具的设计与研究

本项目为北京交通大学机械工程专业本科毕业设计，内容为三自由度教学机械臂“PUMINX”的完整设计与实现过程。该机械臂主要用于教学与展示，具备遥操作与示教复现功能。

---

## 📌 项目信息

- **机械臂本体**：宇树 M8010-6 关节电机  
- **主控**：Jetson Nano  
- **关节采集器**：AD 电位器  
- **采集端主控**：STM32C8T6  
- **通信方式**：UART 串口通信

---

## 📁 仓库内容

| 序号 | 内容描述 | 链接/文件 |
|------|-----------|------------|
| 1 | 📄 本科毕设论文 PDF（仅作学术交流学习使用）<br>⚠️ 温馨提示：请保持学术诚信，具体可详见学术规范相关法律法规、条例 | `./thesis.pdf` |
| 2 | 🎥 末端直线运动功能演示代码 `line.cpp` | [Bilibili 视频](https://www.bilibili.com/video/BV1Qn6nYnE8A/?vd_source=d441e6f9ef7f14877074938825624c3d) |
| 3 | 🎥 双臂遥操作中，**同构机械臂**电位器采集代码（STM32 程序） | [Bilibili 视频](https://www.bilibili.com/video/BV18sRRY2EgL/?vd_source=d441e6f9ef7f14877074938825624c3d) |
| 4 | 📂 `shijiao.cpp`：双臂遥操作中机械臂本体部分代码 | `./shijiao.cpp` |
| 5 | 📂 `shijiaomem.cpp`：加入示教功能的遥操作代码（记录与复现轨迹） | `./shijiaomem.cpp` |
| 6 | 📊 答辩用演示 PPT 文件 | `./PPT.pdf` 或 `./答辩PPT.pptx` |


## 联系方式
如对本项目有兴趣交流：

作者：张海斌

博客：www.haibinzone.cn

邮箱：www.zhb0032024@163.com
