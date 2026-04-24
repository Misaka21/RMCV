# param 模块说明

本目录负责配置读取和运行时参数热重载。

## 两类参数

- 静态参数：启动或初始化时读取，适合相机标定、固定文件路径等。
- 运行时参数：TOML 热重载，适合阈值、滤波噪声、火控开关和调试项。

## 运行时参数硬规则

必须在使用点直接读取：

```cpp
double value = runtime_param::get_param<double>("AutoAim.FireControl.foo");
```

禁止这些写法：

- 构造函数里读取后保存成成员变量。
- `init()` 里读取后长期缓存。
- 用 `static` 局部变量缓存。
- 封装成只加载一次的 helper。
- 做一个 config struct 启动时整体加载。

原因：这些写法会让热重载失效。

## TOML 类型

- `2.8` 对应 `double`
- `2` 对应 `int64_t`
- `false` 对应 `bool`
- 字符串必须加引号

读取类型不匹配时会返回默认值并输出 ERROR 日志。

## 修改参数系统时

同步检查：

- `runtime_parameter.*`
- `static_config.hpp`
- `config/*.toml`
- 所有 `runtime_param::get_param` 调用点
- `test_param`

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_param
```
