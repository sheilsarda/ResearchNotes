# Embedded compute platforms for robotics: Jetson AGX Orin leads for multi-sensor manipulation

The **NVIDIA Jetson AGX Orin** emerges as the optimal embedded compute platform for an 8-DOF arm with mobile base, tactile sensors, and vision-based ML—offering **275 TOPS AI performance**, native dual CAN-FD, and proven RT-PREEMPT Linux support. For budget-conscious builds, the Jetson Orin NX at **157 TOPS** provides 90% of the capability at 40% of the cost. The key differentiator is ML inference: running Digit360, AnySkin, and multi-camera perception at 30-60 FPS requires at minimum **50-100 TOPS**, which eliminates most alternatives except NVIDIA's Orin family and Qualcomm RB6 with AI mezzanine. Industry adoption confirms this—Stanford's ALOHA, Hello Robot Stretch 3, and Boston Dynamics Spot all rely on either Intel NUC (for CPU tasks) or Jetson (for GPU-accelerated perception).

## Platform comparison reveals clear tiers for robotics compute

The embedded compute landscape divides into three performance tiers relevant to this application. The **high-performance tier** (100+ TOPS) includes Jetson AGX Orin and Orin NX, which handle simultaneous multi-camera inference, tactile processing, and sensor fusion. The **mid-tier** (13-26 TOPS) covers Raspberry Pi 5 with Hailo-8 accelerator and Qualcomm RB5/RB6, adequate for simpler vision tasks but struggling with multi-stream tactile ML. The **specialized tier** includes BeagleBone AI-64 with only 8 TOPS but exceptional real-time I/O through its **16 native CAN-FD interfaces** and PRU subsystem capable of sub-10μs latency.

| Platform | AI Performance | Native CAN | USB 3.0+ Ports | Power (TDP) | Price |
|----------|---------------|------------|----------------|-------------|-------|
| **Jetson AGX Orin 64GB** | 275 TOPS | 2× CAN-FD | 3× Gen 2 | 15-60W | $1,599 |
| **Jetson Orin NX 16GB** | 157 TOPS | 1× CAN | 3× Gen 2 | 10-25W | $599 |
| **Jetson Orin Nano 8GB** | 67 TOPS | Via carrier | 3× Gen 2 | 7-25W | $249 |
| Qualcomm RB6 (w/ AIC100) | 70-200 TOPS | None | 2× | ~7W+module | ~$1,000 |
| BeagleBone AI-64 | 8 TOPS | **16× CAN-FD** | 3× | 10-15W | $189 |
| Intel NUC 13 Pro i7 | ~23 TOPS (CPU+iGPU) | None | 3× Gen 2 + TB4 | 28-64W | $700 |
| Raspberry Pi 5 + Hailo-8 | 26 TOPS | None | 2× | 8-15W | $150 |

The Jetson platforms dominate ML inference benchmarks. AGX Orin achieves **300+ FPS on ResNet-50** with TensorRT optimization, while Orin NX delivers 824 FPS on the same model. For context, running Digit360 tactile processing (ResNet-18 backbone) at 60 FPS on six fingertips requires approximately 360 FPS aggregate throughput—well within Orin NX capability but impossible on platforms below 50 TOPS.

## Peripheral availability determines system architecture

Your sensor suite demands specific I/O capabilities that not all platforms provide natively. The most critical requirement is **CAN bus for motor control**—industry-standard for robotic arms and mobile bases running at 500Hz-1kHz control loops.

**CAN bus support varies dramatically:**
- BeagleBone AI-64: 16× MCAN with CAN-FD (best native support)
- Jetson AGX Orin: 2× CAN-FD native (sufficient for most configurations)
- Jetson Orin NX: 1× CAN (may require USB-CAN adapter for complex setups)
- Intel NUC, Raspberry Pi, Qualcomm: None native—requires USB-CAN adapters like PEAK PCAN or Innomaker

**USB bandwidth for cameras** presents another constraint. USB 3.0 provides 5Gbps theoretical, ~1.5-2Gbps practical per root controller. A single 1080p30 uncompressed camera requires ~1.5Gbps. Multiple tactile sensors (Digit360 streams via USB-C, AnySkin via USB) combined with stereo cameras can exhaust bandwidth. Jetson platforms support 3× USB 3.2 Gen 2 (10Gbps each) plus up to **6-16 MIPI CSI cameras** via the sensor processor—critical for eliminating USB bandwidth bottlenecks.

| Peripheral | Requirement | AGX Orin | Orin NX | BeagleBone AI-64 | Intel NUC 13 |
|------------|-------------|----------|---------|------------------|--------------|
| CAN interfaces | 1-2 for arm + base | 2× native | 1× native | 16× native | USB adapter |
| I2C buses | 2-4 for sensors | 8× | 4+ | 10× + 3× I3C | USB adapter |
| CSI cameras | 2-4 recommended | 6-16× | 4× | 2× | None |
| USB 3.0 ports | 3-4 minimum | 3× (10Gbps) | 3× (10Gbps) | 3× (5Gbps) | 3× + 2× TB4 |
| GPIO | 20+ pins | 40-pin header | 40-pin | Extensive | None |
| Ethernet | GbE minimum | 4× 10GbE | 1× GbE | GbE + TSN | 2.5GbE |

Carrier boards significantly expand Jetson I/O. The **Seeed A608** adds dual GbE, 4× USB 3.2, dedicated CAN port, and dual CSI for ~$150. For AGX Orin, the **reServer Industrial J501** provides 10GbE LAN and up to 8× GMSL cameras with hardware synchronization.

## Sensor sampling rates establish control loop requirements

Real-time control of an 8-DOF arm with torque sensors requires understanding minimum viable sampling rates. Joint-level control loops typically run at **1kHz** (matching Franka Emika and KUKA iiwa standards), while outer perception loops can operate at 30-60Hz.

| Sensor Type | Minimum Rate | Typical Rate | Maximum Rate | Interface |
|-------------|--------------|--------------|--------------|-----------|
| Optical encoders | 1 kHz | 10-50 kHz | 4 MHz | SPI, BiSS, SSI |
| Joint torque sensors | 500 Hz | 1-3 kHz | 8 kHz | EtherCAT, CAN |
| IMU | 100 Hz | 200-400 Hz | 4 kHz | SPI, I2C |
| 2D LiDAR | 5 Hz | 10-15 Hz | 100 Hz | USB, Ethernet |
| 3D LiDAR | 5 Hz | 10-20 Hz | 20 Hz | Ethernet |
| Stereo cameras | 10 fps | 30 fps | 120 fps | USB 3.0, CSI |
| Vision-based tactile | 15 fps | 30-60 fps | 60+ fps | USB 2.0/3.0 |
| 6-axis F/T sensors | 100 Hz | 500 Hz-2 kHz | 8 kHz | EtherCAT, Ethernet |

The aggregate data bandwidth is substantial. 3D LiDAR alone demands **10-100 Mbps** (Velodyne VLP-16 generates 300,000-600,000 points/second). Stereo cameras at 720p30 require ~500 Mbps each. Your configuration with LiDAR, stereo cameras, monocular cameras, and multiple tactile sensors likely requires **2-4 Gbps aggregate bandwidth**—manageable on Jetson platforms with CSI + USB + Ethernet distribution.

## Vision-based tactile sensors demand substantial ML compute

Digit360, AnySkin, and UMI represent three distinct tactile sensing paradigms with different compute requirements. **Digit360** (Meta FAIR) is the most demanding—its ~8.3 million taxels stream via USB-C at 30-60 FPS, requiring neural network inference for contact force estimation. The platform includes an on-device AI accelerator for low-latency reflexes, but the **Sparsh-X model** (transformer-based, trained on 1M+ interactions) requires external GPU compute for policy inference.

**AnySkin** uses magnetic sensing rather than vision, making it computationally lighter—raw Hall sensor signals feed directly into lightweight neural networks for slip detection and force estimation. Its key advantage is **zero-shot policy transfer** across sensor instances (only 13% performance degradation vs. 43% for predecessor ReSkin), reducing calibration burden.

**UMI (Universal Manipulation Interface)** employs diffusion policies requiring ~100ms inference on RTX 3080-class GPUs. The ResNet-18 encoder contributes only ~1.3ms, but the denoising diffusion process dominates latency. Recent **Consistency Policy** variants achieve 10× faster inference, potentially enabling embedded deployment on AGX Orin.

| Tactile System | Processing Model | Compute Requirement | Demonstrated Platform |
|----------------|------------------|--------------------|-----------------------|
| Digit360 + Sparsh-X | Transformer encoder | 100+ TOPS for multi-finger | Allegro Hand + GPU workstation |
| AnySkin | Lightweight MLP/CNN | 10-20 TOPS sufficient | xArm, Franka, Leap Hand |
| UMI Diffusion Policy | DDPM (10 denoising steps) | RTX 3080 (desktop GPU) | UR5, Franka |

For running CNNs on tactile camera feeds at 30-60 FPS, benchmarks indicate:
- ResNet-18 on Jetson AGX Orin: **274+ FPS** (FP16, TensorRT optimized)
- ResNet-50 on Orin NX: **~200 FPS** (FP16)
- MobileNetV3 on Orin Nano: **60+ FPS** (sufficient for single sensor)

Processing multiple tactile sensors simultaneously requires **batching inputs** through a shared encoder rather than running duplicate models—Orin platforms handle 4-8 sensor streams with proper pipeline parallelism.

## Real-time OS compatibility favors Jetson and BeagleBone

RT-PREEMPT Linux achieves **30-200μs worst-case latency** on most platforms—acceptable for 1kHz control loops. NVIDIA provides official RT kernel patches for JetPack 6.x, and community reports confirm stable operation. Xenomai support exists but requires custom kernel compilation.

| Platform | RT-PREEMPT | Xenomai | Hard RTOS | Typical Latency |
|----------|------------|---------|-----------|-----------------|
| Jetson AGX Orin | Official support | Community | No | 50-100μs |
| Jetson Orin NX | Official support | Community | No | 50-100μs |
| BeagleBone AI-64 | Supported | Supported | **PRU: \<10μs** | \<10μs on PRU |
| Intel NUC 13 | Possible | Limited | No | 50-200μs |
| Raspberry Pi 5 | Supported | Emerging | No | 30-200μs |
| Qualcomm RB5/RB6 | Not official | No | No | Unknown |

**BeagleBone AI-64 excels for hard real-time**: its dual PRU-ICSSG subsystems (200MHz, 32-bit processors with single-cycle I/O access) achieve microsecond-level determinism impossible on Linux. The Cortex-R5F MCUs can run FreeRTOS for safety-critical motor control while Linux handles perception on Cortex-A72 cores.

ROS 2 real-time support varies. **NVIDIA Isaac ROS** provides optimized perception packages accelerated with CUDA/TensorRT. Standard ROS 2 executors support RT scheduling with appropriate Linux configuration. The key limitation is that ROS 2's DDS middleware introduces latency—critical inner control loops should bypass ROS for direct hardware access.

## Industry architectures converge on proven patterns

Research labs and commercial platforms reveal three dominant architecture patterns for systems similar to your specification:

**Pattern 1: Single powerful compute (most common)**
Stanford's ALOHA uses a System76 laptop (i9 + RTX 4060) or Meerkat NUC. Hello Robot Stretch 3 uses Intel NUC 12 (i5-1240P). This approach prioritizes simplicity—all sensors connect to one machine running ROS 2, with perception and control in the same process. Works well up to ~6-8 DOF with moderate ML requirements.

**Pattern 2: Distributed RT + perception**
Franka Emika Panda separates a dedicated RT controller (1kHz servo loops via EtherCAT) from a workstation PC running planning and perception. Boston Dynamics Spot uses proprietary onboard control with optional Jetson Xavier NX payload for custom vision tasks. This pattern enables deterministic motor control without Linux latency concerns.

**Pattern 3: Onboard + remote GPU**
Stretch 3 and many research setups use lightweight onboard compute (NUC/Jetson) with WiFi/Ethernet offload to a desktop GPU for heavy inference. Dobb-E explicitly recommends RTX 4090 for training and inference. This pattern maximizes ML capability without weight/power penalties but introduces network latency (~10-50ms).

For your 8-DOF arm + mobile base + tactile sensors, industry precedent suggests either:
- **Jetson AGX Orin as single compute** if on-device inference is required
- **Intel NUC + remote GPU** if power budget allows desktop-class compute nearby
- **BeagleBone AI-64 (RT control) + Jetson Orin NX (perception)** for maximum real-time determinism

## Recommendations for your specific configuration

Given your requirements—8 DOF arm, multiple camera types, vision-based tactile sensors at 30-60 FPS, real-time control capability—the optimal solution depends on your constraints:

**Primary recommendation: NVIDIA Jetson AGX Orin 64GB**
- Handles all ML inference on-device (275 TOPS covers multi-sensor tactile + vision)
- Native dual CAN-FD for motor control
- Sufficient USB + CSI bandwidth for all cameras
- RT-PREEMPT proven for 1kHz control
- Used in production by Boston Dynamics (Xavier generation), validated architecture
- Cost: ~$2,000 with carrier board

**Budget alternative: NVIDIA Jetson Orin NX 16GB**
- 157 TOPS handles your workload with proper optimization (TensorRT INT8)
- Requires carrier board with CAN (Seeed J401/A608)
- May need to prioritize ML models or accept slightly lower frame rates
- Cost: ~$750 with carrier board

**Maximum real-time performance: Hybrid architecture**
- BeagleBone AI-64 for motor control (16× CAN-FD, PRU \<10μs latency)
- Jetson Orin NX for perception and ML inference
- EtherCAT or CAN bridge between systems
- Cost: ~$950 total, but increased integration complexity

**Avoid for this application:**
- Qualcomm RB5/RB6: No native CAN, limited RTOS support, smaller ecosystem
- Raspberry Pi 5 + accelerator: USB bandwidth bottleneck with multiple cameras
- Intel NUC alone: Insufficient ML performance without external GPU

## Conclusion

The NVIDIA Jetson AGX Orin represents the most complete solution for your multi-sensor robotic manipulation platform. Its combination of **275 TOPS inference capability, native CAN-FD, extensive camera interfaces, and validated ROS 2 support** addresses every stated requirement without architectural compromises. The platform's adoption by leading research groups (ALOHA, Spot integrations) and Meta FAIR's Digit360 ecosystem provides assurance of long-term viability. For teams prioritizing cost, the Orin NX with an appropriate carrier board delivers ~90% of the capability at 40% of the cost—the most pragmatic choice for academic research. The key architectural decision is whether to run everything on a single Jetson or separate real-time motor control (BeagleBone PRU) from perception compute; the latter adds complexity but guarantees deterministic 1kHz+ control loops independent of perception load.