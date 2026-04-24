# umt 模块说明

`umt/` 是线程间通信基础设施。它被全项目使用，修改影响面很大。

## 主要组件

- `Message.hpp`: 发布订阅消息。
- `ObjManager.hpp`: 类对象管理。
- `BasicObjManager.hpp`: 基础类型和结构体共享对象管理。
- `umt.hpp`: 汇总头文件。

## 使用约定

消息流：

```cpp
umt::Publisher<T> pub("channel");
umt::Subscriber<T> sub("channel");
```

共享状态：

```cpp
auto obj = umt::BasicObjManager<T>::find_or_create("name");
```

如果类型支持 `store()` / `load()`，跨线程读写优先使用它们，避免读到半更新状态。

## 改动规则

- 不要让 UMT 依赖业务模块。
- 异常类型和阻塞语义要保持兼容，调用点大量依赖 `pop_for()` 超时逻辑。
- 修改锁、队列、生命周期时必须检查主流程退出路径。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_param
./build/test_hardware
```
