"""实验接口的最小契约（鸭子类型即可，不做强制继承）。

Env 与 Gazebo 适配器实现 reset/step；决策策略实现 decide。实验脚本只依赖
这些方法，不区分 L0/L1/L2/真实仿真。
"""
from typing import Any, Protocol


class Env(Protocol):
    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple: ...

    def step(self, action) -> tuple: ...

    def close(self) -> None: ...


class Policy(Protocol):
    def reset_episode(self) -> None: ...

    def decide(self, *args: Any, **kwargs: Any) -> dict: ...
