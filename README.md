# ToyFOC

`ToyFOC` 是一个用于学习和实验的无刷电机控制库，基于 rust `embedded-hal` 

主要目标是学习 rust 和 FOC，以及制作各种创客玩具

理论上，该库兼容所有支持 `embedded-hal` 库的芯片，已验证 `rp2040`

## 功能

* ✅ SVPWM 控制
* ✅ SPWM 控制
* ✅ 电流环
* ✅ 3路PWM信号
* ❌ 6路PWM信号


## 关联库

#### toyfoc 的 rp2040 实现

https://github.com/kisy/toyfoc-rp2040

#### 基于 `MQTT` 的 `ESP32-C3` 控制程序 

https://github.com/kisy/toyfocy-ctrl

#### 基于 `MQTT` 的 web 控制端程序 
https://github.com/kisy/toyfocy-web
